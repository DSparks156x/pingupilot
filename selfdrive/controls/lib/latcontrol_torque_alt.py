import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.params import Params

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from opendbc.sunnypilot.car.volkswagen.values import VOLKSWAGEN_HCA_LAT_JERK_FACTOR_STEPS, VOLKSWAGEN_HCA_LAT_ACCEL_FACTOR_STEPS

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

# The "Alt" controller
# VW HCA status 7 has no virtual centering force on the
# "virtual" driver when HCA is status 7.
# This means that the assumption lateral acceleration is
# proportional to torque is fundamentally flawed
# Ie, if we are mid turn at 2.5m/s/s, and want to go straight, 0/m/s/s
# then the regulat latcontrol controller would apply 0 torque, and since there is minimal
# centering force, the vehicle would continue turning. Eventually, the error PID will countersteer to center
# but not before killing a cyclist and hitting an oncoming car in the other lane ping ponging.LatControlTorqueAlt

# To rectify this, we assume that the steer torque is primarily proportional to the 
# rate of change of the steering wheel.
# so, to control it, we use lateral accel jerk as the primary feed forward component.
# the suspensions caster will still cause some centering force, so we still use the original lat accel factory, just significantly reduce it.

# at higher speeds, this approach starts to fall apart, as the centering force becomes larger and larger, and the steering effectively becomes more and more sensitive.
# so, we scale back the jerk component, and scale up the lat accel component at higher speeds.

# the pid error controller is theoretically much less necessary now as well, so we have mildly reduced it...
# in the traditional torque controller, without a jerk component, the feed forward only handles the force needed to maintain the turn (fighting centering force)
# The pid error controller then deals with actually initially achievieving the lateral accel.
# the jerk component inherently deals with this, likely much better, so the pid is also reduced.

# because there are now two major "factors", lat accel factor, and jerk factor, we have disabled auto tune... it wouldnt be impossible to auto tune, but it is much more complex.
 
KP = 0.6
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 1.5, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

class LatControlTorqueAlt(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    self.params = Params()
    self.active_prev = False
    self.lat_jerk_factor = 0.0

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # Ignore live learning because torqued.py cannot model an integrator rack.
    # We strictly rely on the static baseline latAccelFactor defined in CarParams,
    # or the user's custom UI override.
    pass

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Override torque params from extension
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    if active and not self.active_prev:
      try:
        idx = int(self.params.get("VolkswagenHCALatJerkFactor") or 0)
        if 0 <= idx < len(VOLKSWAGEN_HCA_LAT_JERK_FACTOR_STEPS):
          self.lat_jerk_factor = VOLKSWAGEN_HCA_LAT_JERK_FACTOR_STEPS[idx]
        else:
          self.lat_jerk_factor = 0.0
      except ValueError:
        self.lat_jerk_factor = 0.0

      try:
        laf_idx = int(self.params.get("VolkswagenHCALatAccelFactor") or 15)
        if 0 <= laf_idx < len(VOLKSWAGEN_HCA_LAT_ACCEL_FACTOR_STEPS):
          self.torque_params.latAccelFactor = VOLKSWAGEN_HCA_LAT_ACCEL_FACTOR_STEPS[laf_idx]
        self.update_limits()
      except ValueError:
        pass
    self.active_prev = active

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    
    # VW PQ HCA 7 rack is an integrator without virtual centering.
    # We use Jerk FF to handle dynamic turn-in and active unwinding on exit.

    # Speed-dependent scaling:
    # 1. Jerk FF handles movement. At high speeds, it's too aggressive (wiggles).
    #    Scale down from 100% at 15m/s (33mph) to 30% at 35m/s (78mph).
    jerk_speed_scaler = np.interp(CS.vEgo, [15.0, 35.0], [1.0, 0.3])

    # 2. Accel FF handles physical centering (caster trail). This increases with speed.
    #    Scale up from 5% at 10m/s (22mph) to 15% at 35m/s (78mph).
    accel_ff_fraction = np.interp(CS.vEgo, [10.0, 35.0], [0.05, 0.15])

    # 3. Adaptive Jerk Filter: Lower cutoff at high speed to smooth jitters.
    #    Cutoff ramps from 1.2Hz at 15m/s to 0.4Hz at 35m/s.
    jerk_cutoff = np.interp(CS.vEgo, [15.0, 35.0], [1.2, 0.4])
    self.jerk_filter.alpha = self.dt / (1 / (2 * np.pi * jerk_cutoff) + self.dt)

    # To achieve FF_torque = K_j * jerk, we must pass (K_j * jerk) to the PID,
    # which will then be multiplied by latAccelFactor at the end.
    equiv_accel_from_jerk = desired_lateral_jerk * self.lat_jerk_factor * jerk_speed_scaler
    steady_state_accel = gravity_adjusted_future_lateral_accel * accel_ff_fraction

    ff = steady_state_accel + equiv_accel_from_jerk - self.torque_params.latAccelOffset

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral acceleration torque controller extension updates
      # Overrides pid_log.error and output_torque
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     future_desired_lateral_accel, measurement, lateral_accel_deadzone, gravity_adjusted_future_lateral_accel,
                                                     desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
