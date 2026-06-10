"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.vehicle_sub_layouts.brands.base import BrandSettingsMici
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.selfdrive.ui.mici.widgets.button import BigMultiParamToggle, BigParamControl
from opendbc.car.volkswagen.values import VolkswagenFlags


from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import MousePos
import pyray as rl

class BigNumericParamControl(BigButton):
  def __init__(self, text: str, param: str, options: list[str]):
    super().__init__(text, options[0])
    self._param = param
    self._options = options
    self._params = Params()
    self._start_y = 0.0
    self._is_dragging = False

    val = self._params.get(self._param)
    if val is not None:
      try:
        idx = int(val)
        if 0 <= idx < len(self._options):
          self.set_value(self._options[idx])
      except ValueError:
        pass

  def _handle_mouse_press(self, mouse_pos: MousePos):
    super()._handle_mouse_press(mouse_pos)
    self._start_y = mouse_pos.y
    self._is_dragging = True

  def _handle_mouse_event(self, mouse_event):
    super()._handle_mouse_event(mouse_event)
    if not mouse_event.left_pressed and not mouse_event.left_released and self._is_dragging:
      diff_y = mouse_event.pos.y - self._start_y
      if abs(diff_y) > 40:
        cur_idx = self._options.index(self.value)
        if diff_y > 0: # swipe down = decrease
          new_idx = max(0, cur_idx - 1)
        else: # swipe up = increase
          new_idx = min(len(self._options) - 1, cur_idx + 1)
        
        if new_idx != cur_idx:
          self.set_value(self._options[new_idx])
          self._params.put_nonblocking(self._param, new_idx)
          self._start_y = mouse_event.pos.y # Reset origin to allow continuous swiping

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    self._is_dragging = False
    # If it was just a tap (no drag), we cycle forward
    diff_y = mouse_pos.y - self._start_y
    if abs(diff_y) < 10:
      cur_idx = self._options.index(self.value)
      new_idx = (cur_idx + 1) % len(self._options)
      self.set_value(self._options[new_idx])
      self._params.put_nonblocking(self._param, new_idx)


class VolkswagenSettingsMici(BrandSettingsMici):
  def __init__(self):
    super().__init__()

    self.hca_mode = BigMultiParamToggle(
      tr("HCA Mode"),
      "VolkswagenHCAMode",
      [tr("HCA 5"), tr("HCA 7"), tr("HCA 7 Centering"), tr("HCA 7 Alt Controller"), tr("HCA 7 Map Controller"), tr("HCA 7 PID Controller")]
    )

    self.hca_delta_rate_up = BigMultiParamToggle(
      tr("HCA Delta Rate Up"),
      "VolkswagenHCADeltaRateUp",
      [tr("10"), tr("30"), tr("50"), tr("150"), tr("300")]
    )

    self.hca_delta_rate_down = BigMultiParamToggle(
      tr("HCA Delta Rate Down"),
      "VolkswagenHCADeltaRateDown",
      [tr("10"), tr("30"), tr("50"), tr("150"), tr("300")]
    )

    from opendbc.sunnypilot.car.volkswagen.values import VOLKSWAGEN_HCA_LAT_JERK_FACTOR_STEPS, VOLKSWAGEN_HCA_LAT_ACCEL_FACTOR_STEPS
    self.lat_jerk_factor = BigNumericParamControl(
      tr("Alt Jerk Tuning"),
      "VolkswagenHCALatJerkFactor",
      [tr(f"{v:.1f}") for v in VOLKSWAGEN_HCA_LAT_JERK_FACTOR_STEPS]
    )

    self.lat_accel_factor = BigNumericParamControl(
      tr("Alt LatAccel Tuning"),
      "VolkswagenHCALatAccelFactor",
      [tr(f"{v:.1f}") for v in VOLKSWAGEN_HCA_LAT_ACCEL_FACTOR_STEPS]
    )

    self.experimental_long = BigParamControl(
      tr("Experimental Longitudinal"),
      "AlphaLongitudinalEnabled"
    )

    self.hca_centering_full_authority = BigParamControl(
      tr("HCA Centering Full Authority"),
      "VolkswagenHCACenteringFullAuthority"
    )

    self.items = [
      self.hca_mode,
      self.hca_delta_rate_up,
      self.hca_delta_rate_down,
      self.lat_jerk_factor,
      self.lat_accel_factor,
      self.hca_centering_full_authority,
      self.experimental_long,
    ]

  def update_settings(self):
    platform = None
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
    elif ui_state.CP is not None:
      platform = ui_state.CP.carFingerprint

    if platform is None:
      for item in self.items:
        item.set_visible(False)
      return

    from opendbc.car.volkswagen.values import CAR as VW_CAR
    is_pq = False
    is_long_available = False

    if bundle and platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      is_pq = bool(config.flags & VolkswagenFlags.PQ)
      is_long_available = not (config.flags & VolkswagenFlags.PQ_CC_ONLY)
    elif ui_state.CP is not None:
      is_pq = bool(ui_state.CP.flags & VolkswagenFlags.PQ)
      is_long_available = ui_state.CP.alphaLongitudinalAvailable and not (ui_state.CP.flags & VolkswagenFlags.PQ_CC_ONLY)
    elif platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      is_pq = bool(config.flags & VolkswagenFlags.PQ)
      is_long_available = not (config.flags & VolkswagenFlags.PQ_CC_ONLY)

    self.hca_mode.set_visible(is_pq)
    self.hca_delta_rate_up.set_visible(is_pq)
    self.hca_delta_rate_down.set_visible(is_pq)
    self.lat_jerk_factor.set_visible(is_pq)
    self.hca_centering_full_authority.set_visible(is_pq)
    self.experimental_long.set_visible(is_long_available)
