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


class VolkswagenSettingsMici(BrandSettingsMici):
  def __init__(self):
    super().__init__()

    self.hca_mode = BigMultiParamToggle(
      tr("HCA Mode"),
      "VolkswagenHCAMode",
      [tr("HCA 5"), tr("HCA 7"), tr("HCA 7 Centering")]
    )

    self.hca_delta_rate = BigMultiParamToggle(
      tr("HCA Delta Rate"),
      "VolkswagenHCADeltaRate",
      [tr("±5"), tr("±10"), tr("±30"), tr("±50")]
    )

    self.experimental_long = BigParamControl(
      tr("Experimental Longitudinal"),
      "AlphaLongitudinalEnabled"
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
      is_long_available = is_pq and not (config.flags & VolkswagenFlags.PQ_CC_ONLY)
    elif ui_state.CP is not None:
      is_pq = bool(ui_state.CP.flags & VolkswagenFlags.PQ)
      is_long_available = ui_state.CP.alphaLongitudinalAvailable and not (ui_state.CP.flags & VolkswagenFlags.PQ_CC_ONLY)
    elif platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      is_pq = bool(config.flags & VolkswagenFlags.PQ)
      is_long_available = is_pq and not (config.flags & VolkswagenFlags.PQ_CC_ONLY)

    self.hca_mode.set_visible(is_pq)
    self.hca_delta_rate.set_visible(is_pq)
    self.experimental_long.set_visible(is_long_available)
