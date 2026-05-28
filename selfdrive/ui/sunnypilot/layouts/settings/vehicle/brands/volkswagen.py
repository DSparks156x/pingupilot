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
      [tr("HCA 5"), tr("HCA 7"), tr("HCA 7 Centering")],
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

    self.hca_max_steer = multiple_button_item_sp(
      tr("HCA Max Steer Limit"),
      tr("Adjust the maximum steering torque limit for Volkswagen PQ vehicles. Increasing this scales the standard controller's lateral acceleration factor accordingly. WARNING: Verify your steering rack is flashed to support limits above 300 centi-Nm."),
      [tr("300"), tr("350"), tr("400"), tr("450"), tr("500")],
      button_width=150,
      callback=self._on_hca_max_steer_changed,
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
      self.hca_max_steer,
      self.experimental_long,
    ]

  def _on_hca_max_steer_changed(self, index: int):
    previous_index = int(ui_state.params.get("VolkswagenHCAMaxSteer") or "0")
    if index == previous_index:
      return

    if index > 0:
      from openpilot.system.ui.lib.application import gui_app
      from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog

      def cancel_callback():
        self.hca_max_steer.action_item.set_selected_button(previous_index)

      def confirm_callback():
        ui_state.params.put("VolkswagenHCAMaxSteer", str(index))
        ui_state.params.remove("LiveDelay")
        self.hca_max_steer.action_item.set_selected_button(index)

      dlg = BigConfirmationDialog(
        tr("Scale steer limit above 3.0 Nm? (Confirm rack is flashed)"),
        None,
        confirm_callback=confirm_callback,
        cancel_callback=cancel_callback
      )
      gui_app.push_widget(dlg)
    else:
      ui_state.params.put("VolkswagenHCAMaxSteer", "0")
      ui_state.params.remove("LiveDelay")
      self.hca_max_steer.action_item.set_selected_button(0)

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
    self.hca_max_steer.set_visible(self.is_pq)
    self.experimental_long.set_visible(self.alpha_long_available)

    hca_mode_param = int(ui_state.params.get("VolkswagenHCAMode") or "1")
    self.hca_mode.action_item.set_selected_button(hca_mode_param)

    hca_delta_rate_param = int(ui_state.params.get("VolkswagenHCADeltaRate") or "1")
    self.hca_delta_rate.action_item.set_selected_button(hca_delta_rate_param)

    hca_max_steer_param = int(ui_state.params.get("VolkswagenHCAMaxSteer") or "0")
    self.hca_max_steer.action_item.set_selected_button(hca_max_steer_param)
