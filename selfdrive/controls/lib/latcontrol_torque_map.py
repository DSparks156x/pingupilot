import math
import numpy as np
from collections import deque

from cereal import log
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt

# High-confidence baseline grids for AUDI_TT_MK2 (VW PQ HCA 7)
# These grids are printed dynamically by analyze_steering_dynamics.py
# Copy and paste your custom calibrated grids here:
SPEED_GRID = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0]
CURVATURE_GRID = [0.0, 0.002, 0.005, 0.01, 0.02, 0.04, 0.06, 0.08, 0.10, 0.15]
CURVATURE_RATE_GRID = [0.0, 0.001, 0.003, 0.006, 0.01, 0.02, 0.04, 0.06, 0.08, 0.10]

# 8 speeds x 10 curvatures
HOLDING_MAP = [
  [0.0, 20.0, 50.0, 100.0, 200.0, 350.0, 450.0, 500.0, 500.0, 500.0],
  [0.0, 20.0, 50.0, 100.0, 200.0, 350.0, 450.0, 500.0, 500.0, 500.0],
  [0.0, 25.0, 60.0, 120.0, 240.0, 400.0, 480.0, 500.0, 500.0, 500.0],
  [0.0, 25.0, 60.0, 120.0, 240.0, 400.0, 480.0, 500.0, 500.0, 500.0],
  [0.0, 30.0, 70.0, 140.0, 280.0, 450.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 30.0, 70.0, 140.0, 280.0, 450.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 35.0, 80.0, 160.0, 320.0, 480.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 35.0, 80.0, 160.0, 320.0, 480.0, 500.0, 500.0, 500.0, 500.0],
]

# 8 speeds x 10 rates of curvature (winding)
WIND_MAP = [
  [0.0, 10.0, 30.0, 60.0, 100.0, 180.0, 250.0, 300.0, 300.0, 300.0],
  [0.0, 10.0, 30.0, 60.0, 100.0, 180.0, 250.0, 300.0, 300.0, 300.0],
  [0.0, 8.0, 24.0, 48.0, 80.0, 150.0, 220.0, 280.0, 300.0, 300.0],
  [0.0, 8.0, 24.0, 48.0, 80.0, 150.0, 220.0, 280.0, 300.0, 300.0],
  [0.0, 6.0, 18.0, 36.0, 60.0, 120.0, 180.0, 240.0, 280.0, 300.0],
  [0.0, 6.0, 18.0, 36.0, 60.0, 120.0, 180.0, 240.0, 280.0, 300.0],
  [0.0, 4.0, 12.0, 24.0, 40.0, 80.0, 120.0, 180.0, 220.0, 250.0],
  [0.0, 4.0, 12.0, 24.0, 40.0, 80.0, 120.0, 180.0, 220.0, 250.0],
]

# 8 speeds x 10 rates of curvature (unwinding - lower torque needed due to centering trail support)
UNWIND_MAP = [
  [0.0, 5.0, 15.0, 30.0, 50.0, 90.0, 125.0, 150.0, 150.0, 150.0],
  [0.0, 5.0, 15.0, 30.0, 50.0, 90.0, 125.0, 150.0, 150.0, 150.0],
  [0.0, 4.0, 12.0, 24.0, 40.0, 75.0, 110.0, 140.0, 150.0, 150.0],
  [0.0, 4.0, 12.0, 24.0, 40.0, 75.0, 110.0, 140.0, 150.0, 150.0],
  [0.0, 3.0, 9.0, 18.0, 30.0, 60.0, 90.0, 120.0, 140.0, 150.0],
  [0.0, 3.0, 9.0, 18.0, 30.0, 60.0, 90.0, 120.0, 140.0, 150.0],
  [0.0, 2.0, 6.0, 12.0, 20.0, 40.0, 60.0, 90.0, 110.0, 125.0],
  [0.0, 2.0, 6.0, 12.0, 20.0, 40.0, 60.0, 90.0, 110.0, 125.0],
]

KP = 0.4
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [200, 100, 50, 20, 8.0, 4.0, 2.5, 1.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 4

def interpolate_2d(grid_x, grid_y, table_z, x, y):
  """
  Performs fast bilinear interpolation over the 2D calibration grids.
  """
  x = np.clip(x, grid_x[0], grid_x[-1])
  y = np.clip(y, grid_y[0], grid_y[-1])
  
  idx_x = np.searchsorted(grid_x, x)
  idx_y = np.searchsorted(grid_y, y)
  
  idx_x = max(1, min(idx_x, len(grid_x) - 1))
  idx_y = max(1, min(idx_y, len(grid_y) - 1))
  
  x0, x1 = grid_x[idx_x - 1], grid_x[idx_x]
  y0, y1 = grid_y[idx_y - 1], grid_y[idx_y]
  
  z00 = table_z[idx_x - 1][idx_y - 1]
  z01 = table_z[idx_x - 1][idx_y]
  z10 = table_z[idx_x][idx_y - 1]
  z11 = table_z[idx_x][idx_y]
  
  dx = x1 - x0
  dy = y1 - y0
  
  tx = (x - x0) / dx if dx > 0 else 0.0
  ty = (y - y0) / dy if dy > 0 else 0.0
  
  return float((1.0 - tx) * (1.0 - ty) * z00 + \
               tx * (1.0 - ty) * z10 + \
               (1.0 - tx) * ty * z01 + \
               tx * ty * z11)

class LatControlTorqueMap(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    
    # PID feedback loop operates with reduced Kp to avoid fighting the feedforward maps
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
    
    # Low-pass filter for curvature rate to suppress planner step noise
    self.curvature_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.prev_desired_curvature = 0.0

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # Static mapping relies completely on the hardcoded calibrated grids
    pass

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
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

    # Setpoint alignment based on physical latency lag
    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    # Calculate planned future curvature rate (jerk)
    raw_curvature_rate = (desired_curvature - self.prev_desired_curvature) / self.dt
    desired_curvature_rate = self.curvature_rate_filter.update(raw_curvature_rate)
    self.prev_desired_curvature = desired_curvature

    # 1. Holding Torque Feedforward: lookup speed vs absolute curvature
    holding_torque_mag = interpolate_2d(SPEED_GRID, CURVATURE_GRID, HOLDING_MAP, CS.vEgo, abs(desired_curvature))
    holding_torque = math.copysign(holding_torque_mag, desired_curvature)

    # 2. Asymmetric Transient Feedforward: select wind vs unwind map
    # Wind: curvature magnitude is increasing; Unwind: curvature magnitude is decreasing
    is_wind = abs(desired_curvature) >= abs(self.prev_desired_curvature)
    transient_map = WIND_MAP if is_wind else UNWIND_MAP
    
    transient_torque_mag = interpolate_2d(SPEED_GRID, CURVATURE_RATE_GRID, transient_map, CS.vEgo, abs(desired_curvature_rate))
    transient_torque = math.copysign(transient_torque_mag, desired_curvature_rate)

    # Total 2D Map Feedforward Torque
    ff_torque = holding_torque + transient_torque

    # Convert ff_torque back into lateral acceleration space for the standard PID helper updates
    lat_accel_factor = max(self.torque_params.latAccelFactor, 0.01)
    ff_accel = (ff_torque / lat_accel_factor) - roll_compensation

    if not active:
      output_torque = 0.0
      pid_log.active = False
    else:
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff_accel, freeze_integrator=freeze_integrator)
      
      # Feedforward maps determine 90%+ of the rack torque, PID handles minor residuals
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # Lateral extension updates
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff_accel, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
                                                     future_desired_lateral_accel, measurement, lateral_accel_deadzone, future_desired_lateral_accel - roll_compensation,
                                                     desired_curvature, measured_curvature, steer_limited_by_safety, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_curvature_rate)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

      pid_correction = float(self.pid.p + self.pid.i + self.pid.d)
      # Normalize telemetry contributions for graph renders and diagnostics
      pid_log.latAccelFF = float(holding_torque / lat_accel_factor)
      pid_log.jerkFF = float(transient_torque / lat_accel_factor)
      pid_log.latAccelFactor = float(lat_accel_factor)
      pid_log.jerkFactor = float(1.0)
      pid_log.pidContribution = float(pid_correction / lat_accel_factor)

    return -output_torque, 0.0, pid_log
