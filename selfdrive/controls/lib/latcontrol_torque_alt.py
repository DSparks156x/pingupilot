import math
import numpy as np
import json
import time
from collections import deque

from cereal import log
from openpilot.common.swaglog import cloudlog
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

    self.speed_bins = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0]
    self.accel_ff_bins = None
    self.jerk_ff_bins = None
    self.absolute_bins_initialized = False
      
    self.last_param_save_time = time.monotonic()

  def apply_neighborhood_bleed(self, bins, current_speed, update_value):
    for i, bin_speed in enumerate(self.speed_bins):
      distance = abs(current_speed - bin_speed)
      weight = np.exp(-(distance**2) / (2 * 5.0**2)) # sigma = 5.0 m/s
      bins[i] += update_value * weight

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

    if active != self.active_prev:
      cloudlog.debug(f"ALT_DEBUG: Active transition: {self.active_prev} -> {active}")

    if (active and not self.active_prev) or (active and not self.absolute_bins_initialized):
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
        
      if not self.absolute_bins_initialized:
        cloudlog.info(f"ALT_DEBUG: Initializing bins. HCA Jerk Factor: {self.lat_jerk_factor}, Lat Accel Factor: {self.torque_params.latAccelFactor}")
        cached_params = None
        try:
          cached_params = self.params.get("VolkswagenLiveTorqueAlt")
          cloudlog.info(f"ALT_DEBUG: Cached params type: {type(cached_params)}")
        except Exception as e:
          cloudlog.warning(f"ALT_DEBUG: Failed to get cached params: {e}")
          
        if cached_params is not None:
          try:
            if isinstance(cached_params, (str, bytes)):
               cached_params = json.loads(cached_params)
               cloudlog.info("ALT_DEBUG: Manually parsed JSON string/bytes")
            
            if "accel_ff_bins" in cached_params and "jerk_ff_bins" in cached_params:
              cloudlog.info("ALT_DEBUG: Loading bins from cache")
              self.accel_ff_bins = cached_params["accel_ff_bins"]
              self.jerk_ff_bins = cached_params["jerk_ff_bins"]
            else:
              cloudlog.warning("ALT_DEBUG: Cached params missing keys")
              cached_params = None
          except Exception as e:
            cloudlog.warning(f"ALT_DEBUG: Error parsing cached params: {e}")
            cached_params = None

        if cached_params is None:
          cloudlog.info("ALT_DEBUG: No valid cache, initializing defaults")
          base_lat_jerk = self.lat_jerk_factor
          
          # Default initialization curves — dimensionless, same scale as old interp tables
          # Speeds (m/s):     [0.0,  5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0]
          # Speeds (mph):     [  0,   11,   22,   33,   44,   55,   67,   78,   89]
          
          fractions_accel = np.array([0.05, 0.05, 0.05, 0.07, 0.09, 0.11, 0.13, 0.15, 0.15])
          # Sharp drop-off at highway speeds (>45 mph) to prevent wiggle before it has a chance to learn
          fractions_jerk  = np.array([1.0,  1.0,  1.0,  1.0,  0.8,  0.4,  0.2,  0.1,  0.05])
          
          self.accel_ff_bins = fractions_accel.tolist()
          init_jerk = max(base_lat_jerk, 0.3)  # Never initialize jerk bins to zero even at default param
          self.jerk_ff_bins = (fractions_jerk * init_jerk).tolist()
          cloudlog.info(f"ALT_DEBUG: Initialized defaults. Accel bins: {self.accel_ff_bins}, Jerk bins: {self.jerk_ff_bins}")
          
        self.absolute_bins_initialized = True
        cloudlog.info("ALT_DEBUG: Initialization complete")
        
    self.active_prev = active

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    
    # VW PQ HCA 7 rack is an integrator without virtual centering.
    # We use Jerk FF to handle dynamic turn-in and active unwinding on exit.

    if self.absolute_bins_initialized:
      jerk_speed_scaler = float(np.interp(CS.vEgo, self.speed_bins, self.jerk_ff_bins))
      accel_ff_fraction = float(np.interp(CS.vEgo, self.speed_bins, self.accel_ff_bins))
    else:
      jerk_speed_scaler = 0.0
      accel_ff_fraction = 0.0

    # 3. Adaptive Jerk Filter: Lower cutoff at high speed to smooth jitters.
    #    Cutoff ramps from 1.2Hz at 15m/s to 0.4Hz at 35m/s.
    jerk_cutoff = np.interp(CS.vEgo, [15.0, 35.0], [1.2, 0.4])
    self.jerk_filter.alpha = self.dt / (1 / (2 * np.pi * jerk_cutoff) + self.dt)

    # Bins hold dimensionless FF scaling values (same as old interp tables).
    # The PID output is later multiplied by latAccelFactor in torque_from_lateral_accel,
    # so we do NOT divide by it here — that would double-correct.
    equiv_accel_from_jerk = desired_lateral_jerk * jerk_speed_scaler
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

      pid_correction = float(self.pid.p + self.pid.i + self.pid.d)
      # Log contributions in torque space (-1 to 1) so they sum to ~output_torque
      lat_accel_factor = max(self.torque_params.latAccelFactor, 0.01)
      pid_log.latAccelFF = float(steady_state_accel / lat_accel_factor)
      pid_log.jerkFF = float(equiv_accel_from_jerk / lat_accel_factor)
      pid_log.latAccelFactor = float(accel_ff_fraction)
      pid_log.jerkFactor = float(jerk_speed_scaler)
      pid_log.pidContribution = float(pid_correction / lat_accel_factor)
      
      if self.accel_ff_bins is not None:
        pid_log.accelFactorBins = self.accel_ff_bins
      if self.jerk_ff_bins is not None:
        pid_log.jerkFactorBins = self.jerk_ff_bins

      # Adaptive Feed-Forward Learning (bins are in lat accel space, so use pid_correction directly)
      if active and not freeze_integrator and abs(CS.vEgo) > 5.0 and self.absolute_bins_initialized:
        pid_correction_for_learning = pid_correction  # lat accel space, matches bin units
        
        alpha_accel = 0.0001
        alpha_jerk = 0.0001
        
        # 1. Learn Jerk only during transient maneuvers (high jerk)
        if abs(desired_lateral_jerk) > 0.5:
          delta_jerk_ff = alpha_jerk * pid_correction_for_learning * desired_lateral_jerk
          self.apply_neighborhood_bleed(self.jerk_ff_bins, CS.vEgo, delta_jerk_ff)
        
        # 2. Learn Lat Accel only in steady-state (low jerk) and actual curves (high lat accel)
        if abs(desired_lateral_jerk) < 0.2 and abs(gravity_adjusted_future_lateral_accel) > 0.5:
          delta_accel_ff = alpha_accel * pid_correction_for_learning * gravity_adjusted_future_lateral_accel
          self.apply_neighborhood_bleed(self.accel_ff_bins, CS.vEgo, delta_accel_ff)
        
        self.accel_ff_bins = np.clip(self.accel_ff_bins, 0.0, 5.0).tolist()
        self.jerk_ff_bins = np.clip(self.jerk_ff_bins, 0.0, 10.0).tolist()
        
        current_time = time.monotonic()
        if current_time - self.last_param_save_time > 30.0:
          save_data = {
            "accel_ff_bins": self.accel_ff_bins,
            "jerk_ff_bins": self.jerk_ff_bins
          }
          # Params accepts standard Python dicts for JSON params
          self.params.put_nonblocking("VolkswagenLiveTorqueAlt", save_data)
          self.last_param_save_time = current_time

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
