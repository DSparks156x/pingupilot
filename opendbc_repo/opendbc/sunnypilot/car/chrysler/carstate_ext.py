
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.chrysler.values import RAM_HD, RAM_CARS
from opendbc.sunnypilot.car.chrysler.brake_hold import (
  ACC_ACCEL_THRESHOLD,
  ACC_DECEL_THRESHOLD,
  BrakeHold,
)
from opendbc.sunnypilot.car.chrysler.values_ext import BUTTONS, ChryslerFlagsSP


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.button_events = []
    self.button_states = {button.event_type: False for button in BUTTONS}

    self.brake_hold = BrakeHold(CP, CP_SP)

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    button_events = []
    for button in BUTTONS:
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
      self.button_states[button.event_type] = state
    self.button_events = button_events

    if self.CP.carFingerprint in RAM_HD:
      ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"]

    if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED:
      # The EPS naturally outputs LKAS_STATE = 4 at low speeds. The WP mod passes this through while 
      # disengaged, which normally locks openpilot out from ever engaging. We must suppress this.
      # Because we fixed the active-bit race condition in CarController, we shouldn't hit real 
      # EPS lockouts during engagement anymore, making this suppression safe.
      ret.steerFaultPermanent = False

    # Brake hold: track upstream DAS_3 and feed the state machine.
    # cp_cruise mirrors the convention in carstate.py: cam bus for RAM, pt for non-RAM.
    if self.CP_SP.flags & ChryslerFlagsSP.BRAKE_HOLD:
      cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp
      das_3_counter = int(cp_cruise.vl["DAS_3"]["COUNTER"])
      das_3_acc_decel = cp_cruise.vl["DAS_3"]["ACC_DECEL"]
      acc_decelerating = das_3_acc_decel < ACC_DECEL_THRESHOLD
      acc_accelerating = das_3_acc_decel > ACC_ACCEL_THRESHOLD

      self.brake_hold.update_state(ret, acc_decelerating, acc_accelerating,
                                   das_3_counter, das_3_acc_decel)

      # During an active hold, the DASM has cancelled ACC (ACC_ACTIVE=0) and
      # we are spoofing ACC_ACTIVE=1 in our DAS_3 transmissions. The CAN parser
      # sees both messages and ret.cruiseState would flip-flop at 50Hz. Lock
      # the engagement state to "active and at standstill" while we hold.
      if self.brake_hold.active:
        ret.cruiseState.enabled = True
        ret.cruiseState.standstill = True
      
      ret.brakeHoldActive = self.brake_hold.active
