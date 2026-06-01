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
  [0.0, 4.4, 11.0, 22.0, 44.0, 88.0, 131.9, 175.9, 219.9, 329.8],
  [0.0, 8.0, 20.0, 40.0, 80.0, 160.1, 240.2, 320.2, 400.3, 500.0],
  [0.0, 26.5, 66.2, 132.3, 264.6, 500.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 50.2, 125.5, 251.1, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 60.0, 150.0, 300.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 60.0, 150.0, 300.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 60.0, 150.0, 300.0, 500.0, 500.0, 500.0, 500.0, 500.0, 500.0],
  [0.0, 20.9, 52.4, 104.7, 209.5, 418.9, 500.0, 500.0, 500.0, 500.0],
]  # cNm

# 8 speeds x 10 rates of curvature (winding)
WIND_MAP = [
  [0.0, 0.1, 0.3, 0.6, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0],
  [0.0, 0.3, 1.0, 2.0, 3.4, 6.8, 13.6, 20.4, 27.1, 33.9],
  [0.0, 1.7, 5.2, 10.4, 17.3, 34.7, 69.3, 104.0, 138.7, 173.3],
  [0.0, 1.8, 5.4, 10.9, 18.1, 36.3, 72.5, 108.8, 145.1, 181.4],
  [0.0, 9.9, 29.6, 59.2, 98.7, 197.5, 300.0, 300.0, 300.0, 300.0],
  [0.0, 15.0, 45.0, 90.0, 150.0, 300.0, 300.0, 300.0, 300.0, 300.0],
  [0.0, 15.0, 45.0, 90.0, 150.0, 300.0, 300.0, 300.0, 300.0, 300.0],
  [0.0, 15.0, 45.0, 90.0, 150.0, 300.0, 300.0, 300.0, 300.0, 300.0],
]  # cNm

# 8 speeds x 10 rates of curvature (unwinding)
UNWIND_MAP = [
  [0.0, 0.1, 0.3, 0.6, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0],
  [0.0, 0.7, 1.9, 3.9, 6.5, 12.9, 25.8, 38.8, 51.7, 64.6],
  [0.0, 2.8, 8.3, 16.7, 27.8, 55.5, 111.0, 166.6, 222.1, 277.6],
  [0.0, 3.1, 9.2, 18.5, 30.8, 61.6, 123.2, 184.7, 246.3, 300.0],
  [0.0, 0.8, 2.5, 4.9, 8.2, 16.5, 33.0, 49.4, 65.9, 82.4],
  [0.0, 0.1, 0.3, 0.6, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0],
  [0.0, 0.1, 0.3, 0.6, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0],
  [0.0, 0.1, 0.3, 0.6, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0],
]

HCA_LIMIT = 300.0

KP = 60.0
KI = 22.5

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [30000.0, 15000.0, 7500.0, 3000.0, 1200.0, 600.0, 375.0, 150.0, KP]

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

  return float((1.0 - tx) * (1.0 - ty) * z00 + tx * (1.0 - ty) * z10 + (1.0 - tx) * ty * z01 + tx * ty * z11)


class LatControlTorqueMap(LatControl):
  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()

    # PID feedback loop wrapper kept for cereal logging and extension compatibility
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1 / self.dt)
    self.update_limits()

    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.0] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)

    # Low-pass filter for curvature rate to suppress planner step noise
    self.curvature_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.prev_desired_curvature = 0.0

    # Custom error integral state in lateral acceleration space
    self.error_integral = 0.0

    self.extension = LatControlTorqueExt(self, CP, CP_SP, CI)

  def reset(self):
    super().reset()
    self.error_integral = 0.0

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    # Static mapping relies completely on the hardcoded calibrated grids
    pass

  def update_limits(self):
    self.pid.set_limits(1.0, -1.0)

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo**2

    future_desired_lateral_accel = desired_curvature * CS.vEgo**2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo**2

    # Setpoint alignment based on physical latency lag
    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    # Accumulate lateral acceleration error integral with anti-windup clamping
    if active and not CS.steeringPressed and CS.vEgo >= 5:
      self.error_integral += error * self.dt
      self.error_integral = np.clip(self.error_integral, -2.0, 2.0)
    else:
      self.error_integral = 0.0

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

    ff_accel = ff_torque / HCA_LIMIT

    # 3. Scheduled Feedback Torque in lateral acceleration space:
    # Schedule P and I gains directly in physical torque space (cNm / (m/s^2))
    self.pid.speed = CS.vEgo
    Kp = self.pid.k_p
    Ki = self.pid.k_i

    p_contribution = Kp * error
    i_contribution = Ki * self.error_integral

    if not active:
      output_torque = 0.0
      pid_log.active = False
      normalized_output = 0.0
    else:
      pid_log.error = float(error)

      # Combine feedforward and dynamically scheduled feedback components
      output_torque = ff_torque + p_contribution + i_contribution
      output_torque = np.clip(output_torque, -HCA_LIMIT, HCA_LIMIT)
      normalized_output = output_torque / HCA_LIMIT

      # Sync calculated components back to self.pid to ensure 100% compatibility with extensions and logging
      self.pid.p = p_contribution / HCA_LIMIT
      self.pid.i = i_contribution / HCA_LIMIT
      self.pid.f = ff_accel

      # Lateral extension updates
      pid_log, normalized_output = self.extension.update(
        CS,
        VM,
        self.pid,
        params,
        ff_accel,
        pid_log,
        setpoint,
        measurement,
        calibrated_pose,
        roll_compensation,
        future_desired_lateral_accel,
        measurement,
        lateral_accel_deadzone,
        future_desired_lateral_accel - roll_compensation,
        desired_curvature,
        measured_curvature,
        steer_limited_by_safety,
        normalized_output,
      )

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-normalized_output)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_curvature_rate)
      pid_log.saturated = bool(self._check_saturation(1.0 - abs(normalized_output) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

      pid_correction = float(self.pid.p + self.pid.i + self.pid.d)
      # Normalize telemetry contributions for graph renders and diagnostics
      pid_log.latAccelFF = float(holding_torque / HCA_LIMIT)
      pid_log.jerkFF = float(transient_torque / HCA_LIMIT)
      pid_log.latAccelFactor = float(HCA_LIMIT)
      pid_log.jerkFactor = 1.0
      pid_log.pidContribution = float(pid_correction)

    return -normalized_output, 0.0, pid_log
