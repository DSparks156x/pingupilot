"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.vehicle.brands.base import BrandSettings
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, toggle_item_sp
from opendbc.car.volkswagen.values import VolkswagenFlags


class VolkswagenSettings(BrandSettings):
  def __init__(self):
    super().__init__()
    self.is_pq = False
    self.alpha_long_available = False

    self.hca_mode = multiple_button_item_sp(
      tr("HCA Mode"),
      tr("Select the HCA mode for Volkswagen vehicles."),
      [tr("HCA 5"), tr("HCA 7"), tr("HCA 7 Centering"), tr("HCA 7 Alt Controller"), tr("HCA 7 Map Controller"), tr("HCA 7 PID Controller")],
      button_width=200,
      callback=lambda index: ui_state.params.put("VolkswagenHCAMode", index),
      param="VolkswagenHCAMode",
      inline=False
    )

    self.hca_delta_rate = multiple_button_item_sp(
      tr("HCA Delta Rate"),
      tr("Adjust the HCA delta rate."),
      [tr("±5"), tr("±10"), tr("±30"), tr("±50")],
      button_width=200,
      callback=lambda index: ui_state.params.put("VolkswagenHCADeltaRate", index),
      param="VolkswagenHCADeltaRate",
      inline=False
    )

    self.experimental_long = toggle_item_sp(
      lambda: tr("Experimental Longitudinal"),
      description=lambda: tr("Enable experimental longitudinal control for Volkswagen."),
      initial_state=ui_state.params.get_bool("AlphaLongitudinalEnabled"),
      callback=lambda state: ui_state.params.put_bool("AlphaLongitudinalEnabled", state),
      enabled=lambda: not ui_state.engaged,
    )

    self.items = [
      self.hca_mode,
      self.hca_delta_rate,
      self.experimental_long,
    ]

  def update_settings(self):
    platform = None
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("name")
    elif ui_state.CP is not None:
      platform = ui_state.CP.carFingerprint

    if platform is None:
      for item in self.items:
        item.set_visible(False)
      return

    from opendbc.car.volkswagen.values import CAR as VW_CAR
    self.is_pq = False
    self.alpha_long_available = False

    if bundle and platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      self.is_pq = bool(config.flags & VolkswagenFlags.PQ)
      self.alpha_long_available = not (config.flags & VolkswagenFlags.PQ_CC_ONLY)
    elif ui_state.CP is not None:
      self.is_pq = bool(ui_state.CP.flags & VolkswagenFlags.PQ)
      self.alpha_long_available = ui_state.CP.alphaLongitudinalAvailable and not (ui_state.CP.flags & VolkswagenFlags.PQ_CC_ONLY)
    elif platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      self.is_pq = bool(config.flags & VolkswagenFlags.PQ)
      self.alpha_long_available = not (config.flags & VolkswagenFlags.PQ_CC_ONLY)

    self.hca_mode.set_visible(self.is_pq)
    self.hca_delta_rate.set_visible(self.is_pq)
    self.experimental_long.set_visible(self.alpha_long_available)

    hca_mode_param = int(ui_state.params.get("VolkswagenHCAMode") or "1")
    self.hca_mode.action_item.set_selected_button(hca_mode_param)

    hca_delta_rate_param = int(ui_state.params.get("VolkswagenHCADeltaRate") or "1")
    self.hca_delta_rate.action_item.set_selected_button(hca_delta_rate_param)
