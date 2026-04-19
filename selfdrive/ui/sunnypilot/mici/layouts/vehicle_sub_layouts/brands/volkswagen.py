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

    self.items = []

  def update_settings(self):
    if ui_state.CP is None:
      self.items = []
      return

    items = []
    if ui_state.CP.flags & VolkswagenFlags.PQ:
      items.extend([
        self.hca_mode,
        self.hca_delta_rate,
      ])

    if ui_state.CP.alphaLongitudinalAvailable:
      items.append(self.experimental_long)

    self.items = items
