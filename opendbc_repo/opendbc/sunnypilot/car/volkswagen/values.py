"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntEnum


class VolkswagenHCAMode(IntEnum):
  HCA_5 = 0
  HCA_7 = 1
  HCA_7_CENTERING = 2
  HCA_7_ALT = 3
  HCA_7_MAP = 4


class VolkswagenHCADeltaRate(IntEnum):
  RATE_5 = 0
  RATE_10 = 1
  RATE_30 = 2
  RATE_50 = 3


VOLKSWAGEN_HCA_MODE_MAP = {
  VolkswagenHCAMode.HCA_5: 5,
  VolkswagenHCAMode.HCA_7: 7,
  VolkswagenHCAMode.HCA_7_CENTERING: 7,
  VolkswagenHCAMode.HCA_7_ALT: 7,
  VolkswagenHCAMode.HCA_7_MAP: 7,
}

VOLKSWAGEN_HCA_DELTA_RATE_MAP = {
  VolkswagenHCADeltaRate.RATE_5: 5,
  VolkswagenHCADeltaRate.RATE_10: 10,
  VolkswagenHCADeltaRate.RATE_30: 30,
  VolkswagenHCADeltaRate.RATE_50: 50,
}

VOLKSWAGEN_HCA_LAT_JERK_FACTOR_STEPS = [
  0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
  1.1, 1.2, 1.3, 1.4, 1.5
]

VOLKSWAGEN_HCA_LAT_ACCEL_FACTOR_STEPS = [round(1.0 + i * 0.1, 1) for i in range(31)]
