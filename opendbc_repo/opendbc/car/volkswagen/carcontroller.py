import numpy as np
import struct
import cereal.messaging as messaging
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.volkswagen import mlbcan, mqbcan, pqcan
from opendbc.car.volkswagen.centeringforce import VirtualCenteringForce
from opendbc.car.volkswagen.values import CanBus, CarControllerParams, VolkswagenFlags
from openpilot.common.params import Params

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


class HCAMitigation:
  """
  Manages HCA fault mitigations for VW/Audi EPS racks:
    * Reduces torque by 1 for a single frame after commanding the same torque value for too long
  """

  def __init__(self, CCP):
    self._max_same_torque_frames = CCP.STEER_TIME_STUCK_TORQUE / (DT_CTRL * CCP.STEER_STEP)
    self._same_torque_frames = 0

  def update(self, apply_torque, apply_torque_last):
    if apply_torque != 0 and apply_torque_last == apply_torque:
      self._same_torque_frames += 1
      if self._same_torque_frames > self._max_same_torque_frames:
        apply_torque -= (1, -1)[apply_torque < 0]
        self._same_torque_frames = 0
    else:
      self._same_torque_frames = 0

    return apply_torque


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.CCP = CarControllerParams(CP)
    self.CAN = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.aeb_available = not CP.flags & VolkswagenFlags.PQ

    if CP.flags & VolkswagenFlags.PQ:
      self.CCS = pqcan
      self.CCP.STEER_DELTA_UP = self.CP_SP.volkswagenHCADeltaRateUp
      self.CCP.STEER_DELTA_DOWN = self.CP_SP.volkswagenHCADeltaRateDown
    elif CP.flags & VolkswagenFlags.MLB:
      self.CCS = mlbcan
    else:
      self.CCS = mqbcan

    self.apply_torque_last = 0
    self.gra_acc_counter_last = None
    self.hca_mitigation = HCAMitigation(self.CCP)

    # Virtual centering force for HCA7 + Centering mode (PQ only)
    self.use_virtual_centering = (CP.flags & VolkswagenFlags.PQ) and self.CP_SP.volkswagenHCACentering
    if self.use_virtual_centering:
      self.virtual_centering = VirtualCenteringForce()

    self.params = Params()
    self.lat_active_prev = False

    # Glovebox Pi UI TP2.0 responder initialization
    self.is_pq = bool(CP.flags & VolkswagenFlags.PQ)
    if self.is_pq:
      self.sm = messaging.SubMaster(['modelV2', 'radarState'])
      self.tp2_state = "DISCONNECTED"
      self.tester_id = 0x67A
      self.comma_tx_id = 0x6DA
      self.tp2_bus = 1 if (CP.flags & VolkswagenFlags.NO_EXT_CAN) else 2
      self.last_recv_time = 0.0
      self.last_send_time = 0.0
      self.seq = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if CC.latActive and not self.lat_active_prev:
      try:
        from opendbc.sunnypilot.car.volkswagen.values import VOLKSWAGEN_HCA_DELTA_RATE_MAP
        vw_hca_delta_rate_up = int(self.params.get("VolkswagenHCADeltaRateUp") or 0)
        vw_hca_delta_rate_down = int(self.params.get("VolkswagenHCADeltaRateDown") or 0)
        self.CCP.STEER_DELTA_UP = VOLKSWAGEN_HCA_DELTA_RATE_MAP.get(vw_hca_delta_rate_up, 10)
        self.CCP.STEER_DELTA_DOWN = VOLKSWAGEN_HCA_DELTA_RATE_MAP.get(vw_hca_delta_rate_down, 10)
      except ValueError:
        pass
    self.lat_active_prev = CC.latActive

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      apply_torque = 0
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))

        # Apply virtual centering force: bias toward center, clipped so OP retains full ±STEER_MAX authority
        if self.use_virtual_centering:
          centering_bias = self.virtual_centering.compute(CS.out.steeringAngleDeg, CS.out.vEgo)
          if self.CP_SP.volkswagenHCACenteringFullAuthority:
            # Range expansion math: expand OP request range so it hits STEER_MAX even with bias
            if actuators.torque > 0:
              new_torque = int(round(actuators.torque * (self.CCP.STEER_MAX - centering_bias) + centering_bias))
            else:
              new_torque = int(round(actuators.torque * (self.CCP.STEER_MAX + centering_bias) + centering_bias))
          else:
            new_torque = int(np.clip(new_torque + centering_bias, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
          new_torque = int(np.clip(new_torque, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))

        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)

      apply_torque = self.hca_mitigation.update(apply_torque, self.apply_torque_last)
      hca_enabled = apply_torque != 0
      self.apply_torque_last = apply_torque
      can_sends.append(self.CCS.create_steering_control(self.packer_pt, self.CAN.pt, apply_torque, hca_enabled,
                                                         hca_status=self.CP_SP.volkswagenHCAMode if self.CP.flags & VolkswagenFlags.PQ else 7))

      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
        # to the greatest of actual driver input or 2x openpilot's output (1x openpilot output is not enough to
        # consistently reset inactivity detection on straight level roads). See commaai/openpilot#23274 for background.
        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.CCP.STEER_MAX, self.CCP.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(self.CCS.create_eps_update(self.packer_pt, self.CAN.cam, CS.eps_stock_values, ea_simulated_torque))

    # **** Acceleration Controls ******************************************** #
    if self.CP.openpilotLongitudinalControl and not (self.CP.flags & VolkswagenFlags.PQ_CC_ONLY):
      if self.frame % self.CCP.ACC_CONTROL_STEP == 0:
        acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
        accel = float(np.clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0)
        stopping = actuators.longControlState == LongCtrlState.stopping
        starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
        can_sends.extend(self.CCS.create_acc_accel_control(self.packer_pt, self.CAN.pt, CS.acc_type, CC.longActive, accel,
                                                           acc_control, stopping, starting, CS.esp_hold_confirmation))

      #if self.aeb_available:
      #  if self.frame % self.CCP.AEB_CONTROL_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_control(self.packer_pt, False, False, 0.0))
      #  if self.frame % self.CCP.AEB_HUD_STEP == 0:
      #    can_sends.append(self.CCS.create_aeb_hud(self.packer_pt, False, False))

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0:
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      can_sends.append(self.CCS.create_lka_hud_control(self.packer_pt, self.CAN.pt, CS.ldw_stock_values, CC.latActive,
                                                       CS.out.steeringPressed, hud_alert, hud_control))

    if self.frame % self.CCP.ACC_HUD_STEP == 0 and self.CP.openpilotLongitudinalControl and not (self.CP.flags & VolkswagenFlags.PQ_CC_ONLY):
      lead_distance = 0
      if hud_control.leadVisible and self.frame * DT_CTRL > 1.0:  # Don't display lead until we know the scaling factor
        lead_distance = 512 if CS.upscale_lead_car_signal else 8
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.accFaulted, CC.longActive)
      # FIXME: PQ may need to use the on-the-wire mph/kmh toggle to fix rounding errors
      # FIXME: Detect clusters with vEgoCluster offsets and apply an identical vCruiseCluster offset
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      can_sends.append(self.CCS.create_acc_hud_control(self.packer_pt, self.CAN.pt, acc_hud_status, set_speed,
                                                       lead_distance, hud_control.leadDistanceBars))

    # **** Stock ACC Button Controls **************************************** #

    gra_send_ready = self.CP.pcmCruise and CS.gra_stock_values["COUNTER"] != self.gra_acc_counter_last
    if gra_send_ready and (CC.cruiseControl.cancel or CC.cruiseControl.resume) and not (self.CP.flags & VolkswagenFlags.PQ_CC_ONLY):
      can_sends.append(self.CCS.create_acc_buttons_control(self.packer_pt, self.CAN.ext, CS.gra_stock_values,
                                                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume))

    new_actuators = actuators.as_builder()
    if self.CP_SP.volkswagenHCACenteringFullAuthority and CC.latActive:
      new_actuators.torque = actuators.torque
    else:
      new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    # Glovebox Pi TP2.0 responder
    if self.is_pq:
      can_sends.extend(self.update_tp2(CC, CS, now_nanos))

    self.gra_acc_counter_last = CS.gra_stock_values["COUNTER"]
    self.frame += 1
    return new_actuators, can_sends

  def update_tp2(self, CC, CS, now_nanos):
    sends = []
    now_sec = now_nanos * 1e-9

    # Update SubMaster (modelV2, radarState) non-blockingly
    self.sm.update(0)

    # Intercept incoming raw CAN packets saved in CS
    raw_packets = getattr(CS, 'raw_can_packets', [])

    # 1. Connection timeout monitoring (3.0 seconds)
    if self.tp2_state != "DISCONNECTED" and (now_sec - self.last_recv_time) > 3.0:
      self.tp2_state = "DISCONNECTED"

    # 2. Process incoming packets
    for log_mono_time, frames in raw_packets:
      for address, dat, src in frames:
        if address == 0x67A or address == 0x6DA:
          msg_str = f"[{now_sec:.3f}] TP2 RX: addr=0x{address:X} state={self.tp2_state} data={dat.hex()} bus={src}\n"
          print(msg_str.strip(), flush=True)
          try:
            with open("/tmp/tp2_debug.log", "a") as log_f:
              log_f.write(msg_str)
          except:
            pass

        # Handle Parameter Request (A0) at any time to establish or reset the connection
        if address == 0x67A and len(dat) >= 1 and dat[0] == 0xA0:
          self.tp2_bus = src
          # Send Parameters Response (A1) on Comma TX ID (padded to 8 bytes for safety checks)
          resp_data = bytes([0xA1, 0x0F, 0x8A, 0xFF, 0x4A, 0xFF, 0x00, 0x00])
          sends.append((self.comma_tx_id, resp_data, self.tp2_bus))

          self.tp2_state = "CONNECTED"
          self.last_recv_time = now_sec
          self.last_send_time = now_sec
          self.seq = 0

        # Process keep-alives and disconnects on Comma RX ID when CONNECTED
        elif self.tp2_state == "CONNECTED" and address == self.tester_id and len(dat) >= 1:
          self.tp2_bus = src
          opcode_byte = dat[0]

          # Keep Alive Request (A3) -> reply with Keep Alive Response (A1) (padded to 8 bytes for safety checks)
          if opcode_byte == 0xA3:
            sends.append((self.comma_tx_id, bytes([0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), self.tp2_bus))
            self.last_recv_time = now_sec
            self.last_send_time = now_sec

          # Keep Alive Ack/Response (A1)
          elif opcode_byte == 0xA1:
            self.last_recv_time = now_sec

          # Disconnect (A8) -> reset back to DISCONNECTED
          elif opcode_byte == 0xA8:
            self.tp2_state = "DISCONNECTED"

    # 3. In CONNECTED state, send keep-alive and data messages periodically
    if self.tp2_state == "CONNECTED":
      # Send periodic keep-alive ping (A3) every 2.0 seconds if no send occurred (padded to 8 bytes for safety checks)
      if (now_sec - self.last_send_time) > 2.0:
        sends.append((self.comma_tx_id, bytes([0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]), self.tp2_bus))
        self.last_send_time = now_sec

      # Send data messages at 10Hz (every 10 frames)
      if self.frame % 10 == 0:
        sends.extend(self.send_fast_state(CC, CS, now_sec))
        sends.extend(self.send_path_lanes_state(now_sec))

    if sends:
      for addr, data, bus in sends:
        msg_str = f"[{now_sec:.3f}] TP2 TX: addr=0x{addr:X} state={self.tp2_state} data={data.hex()} bus={bus}\n"
        print(msg_str.strip(), flush=True)
        try:
          with open("/tmp/tp2_debug.log", "a") as log_f:
            log_f.write(msg_str)
        except:
          pass

    return sends

  def send_fast_state(self, CC, CS, now_sec):
    # 1. engaged
    engaged = CC.enabled

    # 2. Extract 3 leads
    lead0_dist = 255
    lead0_lat_dist = 0.0
    lead1_dist = 0
    lead1_lat_dist = 0.0
    lead2_dist = 0
    lead2_lat_dist = 0.0
    lead_detected = False

    # Try radarState first for lead0 and lead1
    if self.sm.seen['radarState']:
      radar = self.sm['radarState']
      if radar.leadOne.status:
        lead_detected = True
        lead0_dist = int(np.clip(radar.leadOne.dRel, 0, 255))
        lead0_lat_dist = float(radar.leadOne.yRel)
      if radar.leadTwo.status:
        lead1_dist = int(np.clip(radar.leadTwo.dRel, 0, 255))
        lead1_lat_dist = float(radar.leadTwo.yRel)

    # Fallback/supplement with modelV2 leadsV3
    if self.sm.seen['modelV2']:
      model = self.sm['modelV2']
      # If lead0 wasn't detected by radar but model sees a lead
      if not lead_detected and len(model.leadsV3) > 0 and model.leadsV3[0].prob > 0.5:
        lead_detected = True
        lead0_dist = int(np.clip(model.leadsV3[0].x[0], 0, 255))
        lead0_lat_dist = float(model.leadsV3[0].y[0])
      # If lead1 wasn't detected by radar but model sees a second lead
      if lead1_dist == 0 and len(model.leadsV3) > 1 and model.leadsV3[1].prob > 0.5:
        lead1_dist = int(np.clip(model.leadsV3[1].x[0], 0, 255))
        lead1_lat_dist = float(model.leadsV3[1].y[0])
      # Lead2 is only in model
      if len(model.leadsV3) > 2 and model.leadsV3[2].prob > 0.5:
        lead2_dist = int(np.clip(model.leadsV3[2].x[0], 0, 255))
        lead2_lat_dist = float(model.leadsV3[2].y[0])

    # If still not detected, fallback to HUD leadVisible
    if not lead_detected and CC.hudControl.leadVisible:
      lead_detected = True
      lead0_dist = 42
      lead0_lat_dist = 0.0

    # 3. model_confidence
    confidence = 1.0
    if self.sm.seen['modelV2']:
      model = self.sm['modelV2']
      if len(model.laneLineProbs) >= 4:
        confidence = float(np.mean(model.laneLineProbs))

    # 4. speed and max_speed
    if not hasattr(self, 'is_metric'):
      self.is_metric = self.params.get_bool("IsMetric")
    elif self.frame % 100 == 0:
      self.is_metric = self.params.get_bool("IsMetric")

    v_ego = CS.out.vEgo
    v_cruise = CS.out.vCruise

    if self.is_metric:
      speed = int(round(v_ego * 3.6))
      max_speed = int(round(v_cruise))
    else:
      speed = int(round(v_ego * 2.23694))
      max_speed = int(round(v_cruise * 0.621371))

    speed = int(np.clip(speed, 0, 255))
    max_speed = int(np.clip(max_speed, 0, 255))

    # 5. steer_angle and steer_torque
    steer_angle_raw = int(np.clip(round(CS.out.steeringAngleDeg / 0.05), -32768, 32767))
    # use commanded torque output: CC.actuators.torque
    steer_torque_raw = int(np.clip(round(CC.actuators.torque / 0.01), -128, 127))

    # Scale lateral offsets
    lead0_lat_dist_raw = int(np.clip(round(lead0_lat_dist / 0.1), -128, 127))
    lead1_dist_raw = int(np.clip(round(lead1_dist), 0, 255))
    lead1_lat_dist_raw = int(np.clip(round(lead1_lat_dist / 0.1), -128, 127))
    lead2_dist_raw = int(np.clip(round(lead2_dist), 0, 255))
    lead2_lat_dist_raw = int(np.clip(round(lead2_lat_dist / 0.1), -128, 127))

    # Pack Fast State message payload (13 bytes)
    flags_conf = (int(engaged) << 7) | (int(lead_detected) << 6) | (int(confidence * 63) & 0x3F)
    payload = struct.pack(
        ">B B B B B h b b B b B b",
        0x01, flags_conf, speed, max_speed, lead_dist, steer_angle_raw, steer_torque_raw,
        lead0_lat_dist_raw, lead1_dist_raw, lead1_lat_dist_raw, lead2_dist_raw, lead2_lat_dist_raw
    )

    # Encapsulate in TP2.0 frames
    f1_header = 0x20 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f1_data = bytes([f1_header, 0x00, 0x0D]) + payload[0:5]

    f2_header = 0x20 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f2_data = bytes([f2_header]) + payload[5:12]

    f3_header = 0x30 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f3_data = bytes([f3_header]) + payload[12:13] + b"\x00\x00\x00\x00\x00\x00"

    self.last_send_time = now_sec
    return [
        (self.comma_tx_id, f1_data, self.tp2_bus),
        (self.comma_tx_id, f2_data, self.tp2_bus),
        (self.comma_tx_id, f3_data, self.tp2_bus)
    ]

  def send_path_lanes_state(self, now_sec):
    model = self.sm['modelV2'] if self.sm.seen['modelV2'] else None

    a_6 = a_5 = a_4 = a_3 = a_2 = a_1 = c_p = 0.0
    c = [0.0] * 4
    d = [0.0] * 2
    p = [0] * 4
    re = [0] * 2

    if model is not None and len(model.position.x) > 0:
      x_plan = np.array(model.position.x)
      y_plan = np.array(model.position.y)
      mask = x_plan <= 30.0
      x_fit = x_plan[mask]
      y_fit = y_plan[mask]

      if len(x_fit) >= 6:
        c_p = y_fit[0]
        y_offset = y_fit - c_p
        X = np.vstack([x_fit**6, x_fit**5, x_fit**4, x_fit**3, x_fit**2, x_fit]).T
        try:
          coeffs, _, _, _ = np.linalg.lstsq(X, y_offset, rcond=None)
          a_6, a_5, a_4, a_3, a_2, a_1 = coeffs
        except Exception:
          pass
      else:
        if len(y_fit) > 0:
          c_p = y_fit[0]

      # Determine lane offsets
      for k in range(4):
        if k < len(model.laneLines):
          x_lane = np.array(model.laneLines[k].x)
          y_lane = np.array(model.laneLines[k].y)
          mask = x_lane <= 30.0
          x_lane_fit = x_lane[mask]
          y_lane_fit = y_lane[mask]
          if len(x_lane_fit) > 0:
            y_base = (a_6 * (x_lane_fit**6) + a_5 * (x_lane_fit**5) + a_4 * (x_lane_fit**4) +
                      a_3 * (x_lane_fit**3) + a_2 * (x_lane_fit**2) + a_1 * x_lane_fit)
            c[k] = float(np.mean(y_lane_fit - y_base))

      # Determine road edge offsets
      for j in range(2):
        if j < len(model.roadEdges):
          x_edge = np.array(model.roadEdges[j].x)
          y_edge = np.array(model.roadEdges[j].y)
          mask = x_edge <= 30.0
          x_edge_fit = x_edge[mask]
          y_edge_fit = y_edge[mask]
          if len(x_edge_fit) > 0:
            y_base = (a_6 * (x_edge_fit**6) + a_5 * (x_edge_fit**5) + a_4 * (x_edge_fit**4) +
                      a_3 * (x_edge_fit**3) + a_2 * (x_edge_fit**2) + a_1 * x_edge_fit)
            d[j] = float(np.mean(y_edge_fit - y_base))

      # Determine lane line probs
      for k in range(4):
        if k < len(model.laneLineProbs):
          p[k] = int(np.clip(round(model.laneLineProbs[k] * 10.0), 0, 15))

      # Determine road edge probs
      if len(model.roadEdges) > 0:
        re[0] = 10
      if len(model.roadEdges) > 1:
        re[1] = 10

    # Scaling & Clamping
    a_6_raw = int(np.clip(round(a_6 / 1e-12), -32768, 32767))
    a_5_raw = int(np.clip(round(a_5 / 1e-10), -32768, 32767))
    a_4_raw = int(np.clip(round(a_4 / 1e-8), -32768, 32767))
    a_3_raw = int(np.clip(round(a_3 / 1e-6), -32768, 32767))
    a_2_raw = int(np.clip(round(a_2 / 1e-5), -32768, 32767))
    a_1_raw = int(np.clip(round(a_1 / 1e-4), -32768, 32767))

    c_p_raw = int(np.clip(round(c_p / 0.1), -128, 127))
    c_0_raw = int(np.clip(round(c[0] / 0.1), -128, 127))
    c_1_raw = int(np.clip(round(c[1] / 0.1), -128, 127))
    c_2_raw = int(np.clip(round(c[2] / 0.1), -128, 127))
    c_3_raw = int(np.clip(round(c[3] / 0.1), -128, 127))
    d_0_raw = int(np.clip(round(d[0] / 0.1), -128, 127))
    d_1_raw = int(np.clip(round(d[1] / 0.1), -128, 127))

    probs_01 = (p[0] << 4) | p[1]
    probs_23 = (p[2] << 4) | p[3]
    probs_re = (re[0] << 4) | re[1]

    # Pack payload
    payload = struct.pack(
        ">B h h h h h h b b b b b b b B B B",
        0x02,
        a_6_raw, a_5_raw, a_4_raw, a_3_raw, a_2_raw, a_1_raw,
        c_p_raw, c_0_raw, c_1_raw, c_2_raw, c_3_raw, d_0_raw, d_1_raw,
        probs_01, probs_23, probs_re
    )

    # Encapsulate in 4 TP2.0 continuation/last frames
    f1_header = 0x20 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f1_data = bytes([f1_header, 0x00, 0x17]) + payload[0:5]

    f2_header = 0x20 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f2_data = bytes([f2_header]) + payload[5:12]

    f3_header = 0x20 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f3_data = bytes([f3_header]) + payload[12:19]

    f4_header = 0x30 | (self.seq & 0x0F)
    self.seq = (self.seq + 1) % 16
    f4_data = bytes([f4_header]) + payload[19:23] + b"\x00\x00\x00"

    self.last_send_time = now_sec
    return [
        (self.comma_tx_id, f1_data, self.tp2_bus),
        (self.comma_tx_id, f2_data, self.tp2_bus),
        (self.comma_tx_id, f3_data, self.tp2_bus),
        (self.comma_tx_id, f4_data, self.tp2_bus)
    ]
