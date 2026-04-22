import numpy as np


class VirtualCenteringForce:
  """
  Emulates HCA5-like centering behavior for HCA7 mode on VW PQ platforms.

  Produces a torque bias (in HCA counts) that pushes steering toward center,
  using a non-linear angle curve with speed-dependent scaling.

  The angle curve emulates real self-aligning torque:
    - Gentle near center (let OP handle fine corrections)
    - Steepest in the 5-30 deg range (where you feel centering)
    - Flattens at large angles (preserve OP authority for curves)

  Speed scaling ramps from zero at standstill to full at highway speed,
  so centering doesn't fight parking maneuvers.
  """

  # Steering angle breakpoints (degrees) and base centering force (HCA counts)
  # Non-linear curve: gentle near center, strong mid-range, plateaus at large angles
  ANGLE_BP = [0.0, 3.0, 10.0, 30.0, 80.0, 150.0]
  FORCE_BP = [0.0, 2.0, 15.0, 40.0, 55.0,  60.0]

  # Speed breakpoints (m/s) and scaling factor (0.0 to 1.0)
  SPEED_BP    = [0.0, 5.0, 15.0, 30.0]
  SPEED_SCALE = [0.0, 0.3,  0.7,  1.0]

  def compute(self, steering_angle_deg: float, v_ego: float) -> int:
    """
    Compute centering torque bias in HCA counts.

    Sign convention: opposes steering angle (pushes toward 0 deg).
    Positive steering angle → negative centering bias.

    Args:
      steering_angle_deg: Current steering wheel angle in degrees
      v_ego: Vehicle speed in m/s

    Returns:
      Centering bias in HCA counts (integer), sign opposes angle
    """
    abs_angle = abs(steering_angle_deg)

    # Look up base centering force from non-linear angle curve
    base_force = float(np.interp(abs_angle, self.ANGLE_BP, self.FORCE_BP))

    # Apply speed-dependent scaling
    speed_scale = float(np.interp(v_ego, self.SPEED_BP, self.SPEED_SCALE))

    # Apply sign: centering opposes the current angle
    centering = base_force * speed_scale
    if steering_angle_deg > 0:
      centering = -centering

    return int(round(centering))