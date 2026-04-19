"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.vehicle_sub_layouts.brands.base import BrandSettingsMici
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.multilang import tr
from openpilot.selfdrive.ui.mici.widgets.button import BigMultiParamToggle, BigParamControl, BigButton
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

    self.diag_platform = BigButton(tr("Diag: Platform"), "", scroll=True)
    self.diag_pq = BigButton(tr("Diag: Is PQ"), "", scroll=True)
    self.diag_long = BigButton(tr("Diag: Is Long"), "", scroll=True)

    self.items = []

  def update_settings(self):
    platform = None
    bundle = ui_state.params.get("CarPlatformBundle")
    if bundle:
      platform = bundle.get("platform")
    elif ui_state.CP is not None:
      platform = ui_state.CP.carFingerprint

    if platform is None:
      self.items = []
      return

    from opendbc.car.volkswagen.values import CAR as VW_CAR
    is_pq = False
    is_long_available = False

    if bundle and platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      is_pq = config.flags & VolkswagenFlags.PQ
      is_long_available = is_pq
      diag_flags = hex(int(config.flags))
    elif ui_state.CP is not None:
      is_pq = ui_state.CP.flags & VolkswagenFlags.PQ
      is_long_available = ui_state.CP.alphaLongitudinalAvailable
      diag_flags = hex(int(ui_state.CP.flags))
    elif platform in VW_CAR.__members__:
      config = VW_CAR[platform].config
      is_pq = config.flags & VolkswagenFlags.PQ
      is_long_available = is_pq
      diag_flags = hex(int(config.flags))
    else:
      diag_flags = "None"

    self.diag_platform.set_value(str(platform))
    self.diag_pq.set_value(f"{bool(is_pq)} ({diag_flags})")
    self.diag_long.set_value(str(is_long_available))

    items = [self.diag_platform, self.diag_pq, self.diag_long]
    if is_pq:
      items.extend([
        self.hca_mode,
        self.hca_delta_rate,
      ])

    if is_long_available:
      items.append(self.experimental_long)

    self.items = items
