# -*- coding: utf-8 -*-
"""
Created on 10 Mar 2022

@author: Philip Winkler, Sabrina Rossberger

Copyright © 2022 Sabrina Rossberger, Delmic

This file is part of Odemis.

Odemis is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License version 2 as published by the Free
Software Foundation.

Odemis is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Odemis. If not, see http://www.gnu.org/licenses/.


### Purpose ###

This module contains classes to control the actions related to the calibration
and alignment of the FASTEM system and classes to control actions related to the
overview image and multibeam acquisition.
"""

from __future__ import division

import logging
import math
import os
from builtins import str
from concurrent.futures._base import CancelledError
from datetime import datetime
from functools import partial

import wx

from odemis import model, dataio
from odemis.acq import align, stream, fastem
from odemis.acq.align import fastem as align_fastem
from odemis.acq.stream import FastEMOverviewStream
from odemis.gui import FG_COLOUR_BUTTON
from odemis.gui.util import get_picture_folder, call_in_wx_main, \
    wxlimit_invocation
from odemis.gui.util.widgets import ProgressiveFutureConnector
from odemis.util import units
from odemis.util.dataio import open_acquisition, data_to_static_streams


class FastEMOverviewAcquiController(object):
    """
    Takes care of the overview image acquisition in the FastEM overview tab.
    """

    def __init__(self, tab_data, tab_panel):
        """
        tab_data (FastEMGUIData): the representation of the microscope GUI
        tab_panel: (wx.Frame): the frame which contains the viewport
        """
        self._tab_data_model = tab_data
        self._main_data_model = tab_data.main
        self._tab_panel = tab_panel

        # For acquisition
        self.btn_acquire = self._tab_panel.btn_sparc_acquire
        self.btn_cancel = self._tab_panel.btn_sparc_cancel
        self.acq_future = None  # ProgressiveBatchFuture
        self._fs_connector = None  # ProgressiveFutureConnector
        self.gauge_acq = self._tab_panel.gauge_sparc_acq
        self.lbl_acqestimate = self._tab_panel.lbl_sparc_acq_estimate
        self.bmp_acq_status_warn = self._tab_panel.bmp_acq_status_warn
        self.bmp_acq_status_info = self._tab_panel.bmp_acq_status_info
        self.selection_panel = self._tab_panel.selection_panel

        # Create grid of buttons for scintillator selection
        self.selection_panel.create_controls(tab_data.main.scintillator_layout)
        for btn in self.selection_panel.buttons.values():
            btn.Bind(wx.EVT_TOGGLEBUTTON, self._on_selection_button)
            btn.Enable(False)  # disabled by default, need to select scintillator in chamber tab first

        self._main_data_model.active_scintillators.subscribe(self._on_active_scintillators)

        # Link acquire/cancel buttons
        self.btn_acquire.Bind(wx.EVT_BUTTON, self.on_acquisition)
        self.btn_cancel.Bind(wx.EVT_BUTTON, self.on_cancel)

        # Hide gauge, disable acquisition button
        self.gauge_acq.Hide()
        self._tab_panel.Parent.Layout()
        self.btn_acquire.Enable(False)

        # Warning that no scintillators are selected
        self.update_acquisition_time()

        # If scanner dwell time is changed, update the estimated acquisition time.
        tab_data.main.ebeam.dwellTime.subscribe(self.update_acquisition_time)

    def _on_selection_button(self, evt):
        # add/remove scintillator number to/from selected_scintillators set and toggle button colour
        btn = evt.GetEventObject()
        num = [num for num, b in self.selection_panel.buttons.items() if b == btn][0]
        if btn.GetValue():
            if num not in self._tab_data_model.selected_scintillators.value:
                self._tab_data_model.selected_scintillators.value.append(num)
            btn.SetBackgroundColour(wx.GREEN)
        else:
            if num in self._tab_data_model.selected_scintillators.value:
                self._tab_data_model.selected_scintillators.value.remove(num)
            btn.SetBackgroundColour(FG_COLOUR_BUTTON)
        self.update_acquisition_time()
        self.check_acquire_button()

    @call_in_wx_main
    def _on_active_scintillators(self, evt):
        for num, b in self.selection_panel.buttons.items():
            if num in self._main_data_model.active_scintillators.value:
                b.Enable(True)
            else:
                b.Enable(False)
                if num in self._tab_data_model.selected_scintillators.value:
                    self._tab_data_model.selected_scintillators.value.remove(num)
        self.update_acquisition_time()
        self.check_acquire_button()

    @call_in_wx_main
    def check_acquire_button(self):
        self.btn_acquire.Enable(True if self._tab_data_model.selected_scintillators.value else False)

    def update_acquisition_time(self, _=None):
        lvl = None  # icon status shown
        if not self._main_data_model.active_scintillators.value:
            lvl = logging.WARN
            txt = "No scintillator loaded (go to Chamber tab)."
        elif not self._tab_data_model.selected_scintillators.value:
            lvl = logging.WARN
            txt = "No scintillator selected for overview acquisition."
        else:
            acq_time = 0
            # Add up the acquisition time of all the selected scintillators
            for num in self._tab_data_model.selected_scintillators.value:
                center = self._tab_data_model.main.scintillator_positions[num]
                sz = self._tab_data_model.main.scintillator_size
                coords = (center[0] - sz[0] / 2, center[1] - sz[1] / 2,
                          center[0] + sz[0] / 2, center[1] + sz[1] / 2)
                acq_time += fastem.estimateTiledAcquisitionTime(self._tab_data_model.streams.value[0],
                                                                self._main_data_model.stage, coords)

            acq_time = math.ceil(acq_time)  # round a bit pessimistic
            txt = u"Estimated time is {}."
            txt = txt.format(units.readable_time(acq_time))
        logging.debug("Updating status message %s, with level %s", txt, lvl)
        self._set_status_message(txt, lvl)

    @call_in_wx_main
    def _reset_acquisition_gui(self, text=None, level=None):
        """
        Set back every GUI elements to be ready for the next acquisition
        text (None or str): a (error) message to display instead of the
          estimated acquisition time
        level (None or logging.*): logging level of the text, shown as an icon.
          If None, no icon is shown.
        """
        self.btn_cancel.Hide()
        self.btn_acquire.Enable()
        self._tab_panel.Layout()

        if text is not None:
            self._set_status_message(text, level)
        else:
            self.update_acquisition_time()

    @wxlimit_invocation(1)
    def _set_status_message(self, text, level=None):
        self.lbl_acqestimate.SetLabel(text)
        # update status icon to show the logging level
        self.bmp_acq_status_info.Show(level in (logging.INFO, logging.DEBUG))
        self.bmp_acq_status_warn.Show(level == logging.WARN)
        self._tab_panel.Layout()

    def on_acquisition(self, evt):
        """
        Start the acquisition (really)
        """
        self.update_acquisition_time()  # make sure we show the right label if the previous acquisition failed
        self._main_data_model.is_acquiring.value = True
        self.btn_acquire.Enable(False)
        self.btn_cancel.Enable(True)
        self.btn_cancel.Show()
        self.gauge_acq.Show()

        self.gauge_acq.Range = len(self._tab_data_model.selected_scintillators.value)
        self.gauge_acq.Value = 0

        # Acquire ROAs for all projects
        acq_futures = {}
        for num in self._tab_data_model.selected_scintillators.value:
            center = self._tab_data_model.main.scintillator_positions[num]
            sz = self._tab_data_model.main.scintillator_size
            coords = (center[0] - sz[0] / 2, center[1] - sz[1] / 2,
                      center[0] + sz[0] / 2, center[1] + sz[1] / 2)
            try:
                f = fastem.acquireTiledArea(self._tab_data_model.streams.value[0], self._main_data_model.stage, coords)
                t = fastem.estimateTiledAcquisitionTime(self._tab_data_model.streams.value[0],
                                                        self._main_data_model.stage, coords)
            except Exception:
                logging.exception("Failed to start overview acquisition")
                # Try acquiring the other
                continue

            f.add_done_callback(partial(self.on_acquisition_done, num=num))
            acq_futures[f] = t

        if acq_futures:
            self.acq_future = model.ProgressiveBatchFuture(acq_futures)
            self.acq_future.add_done_callback(self.full_acquisition_done)
            self._fs_connector = ProgressiveFutureConnector(self.acq_future, self.gauge_acq, self.lbl_acqestimate)
        else:  # In case all acquisitions failed to start
            self._main_data_model.is_acquiring.value = False
            self._reset_acquisition_gui("Acquisition failed (see log panel).", level=logging.WARNING)

    def on_cancel(self, evt):
        """
        Called during acquisition when pressing the cancel button
        """
        if not self.acq_future:
            msg = "Tried to cancel acquisition while it was not started"
            logging.warning(msg)
            return

        self.acq_future.cancel()
        # all the rest will be handled by on_acquisition_done()

    def on_acquisition_done(self, future, num):
        """
        Callback called when the one overview image acquisition is finished.
        """
        try:
            da = future.result()
        except CancelledError:
            self._reset_acquisition_gui()
            return
        except Exception:
            # leave the gauge, to give a hint on what went wrong.
            logging.exception("Acquisition failed")
            self._reset_acquisition_gui("Acquisition failed (see log panel).", level=logging.WARNING)
            return

        # Store DataArray as TIFF in pyramidal format and reopen as static stream (to be memory-efficient)
        # TODO: pick a different name from previous acquisition?
        fn = os.path.join(get_picture_folder(), "fastem_overview_%s.ome.tiff" % num)
        dataio.tiff.export(fn, da, pyramid=True)
        da = open_acquisition(fn)
        s = data_to_static_streams(da)[0]
        s = FastEMOverviewStream(s.name.value, s.raw[0])
        # Dict VA needs to be explicitly copied, otherwise it doesn't detect the change
        ovv_ss = self._main_data_model.overview_streams.value.copy()
        ovv_ss[num] = s
        self._main_data_model.overview_streams.value = ovv_ss

    @call_in_wx_main
    def full_acquisition_done(self, future):
        """
        Callback called when the acquisition of all selected overview images is finished
        (either successfully or cancelled).
        """
        self.btn_cancel.Hide()
        self.btn_acquire.Enable()
        self.gauge_acq.Hide()
        self._tab_panel.Layout()
        self._set_status_message("Acquisition done.", logging.INFO)
        self._main_data_model.is_acquiring.value = False


class FastEMAcquiController(object):
    """
    Takes care of the acquisition button and process in the FastEM acquisition tab.
    """

    def __init__(self, tab_data, tab_panel, projectbar_ctrl, calibration_ctrl):
        """
        tab_data (FastEMGUIData): the representation of the microscope GUI
        tab_panel: (wx.Frame): the frame which contains the viewport
        projectbar_ctrl (FastEMProjectBarController): project bar controller
        calibration_ctrl (FastEMCalibrationController): calibration controller
        """
        self._tab_data_model = tab_data
        self._main_data_model = tab_data.main
        self._tab_panel = tab_panel
        self._projectbar_ctrl = projectbar_ctrl
        self._calibration_ctrl = calibration_ctrl

        # Path to the acquisition
        self.path = datetime.today().strftime('%Y-%m-%d')
        self._tab_panel.txt_destination.SetValue(self.path)

        # ROA count
        self.roa_count = 0
        self._tab_panel.txt_num_rois.SetValue("0")

        # For acquisition
        self.btn_acquire = self._tab_panel.btn_sparc_acquire
        self.btn_cancel = self._tab_panel.btn_sparc_cancel
        self.gauge_acq = self._tab_panel.gauge_sparc_acq
        self.lbl_acqestimate = self._tab_panel.lbl_sparc_acq_estimate
        self.txt_num_rois = self._tab_panel.txt_num_rois
        self.bmp_acq_status_warn = self._tab_panel.bmp_acq_status_warn
        self.bmp_acq_status_info = self._tab_panel.bmp_acq_status_info
        self.acq_future = None  # ProgressiveBatchFuture
        self._fs_connector = None  # ProgressiveFutureConnector

        # Link buttons
        self.btn_acquire.Bind(wx.EVT_BUTTON, self.on_acquisition)
        self.btn_cancel.Bind(wx.EVT_BUTTON, self.on_cancel)

        # Hide gauge, disable acquisition button
        self.gauge_acq.Hide()
        self._tab_panel.Parent.Layout()
        self.btn_acquire.Enable(False)

        # Update text controls when projects/roas/rocs are changed
        self.roa_subscribers = []  # list of ROA subscribers (to make sure we don't subscribe to the same ROA twice)
        tab_data.projects.subscribe(self._on_projects, init=True)
        for roc in self._tab_data_model.calibration_regions.value.values():
            roc.coordinates.subscribe(self._on_va_change)

        self._main_data_model.is_aligned.subscribe(self._on_va_change, init=True)
        self._main_data_model.is_acquiring.subscribe(self._on_va_change)

    def _on_projects(self, projects):
        for p in projects:
            p.roas.subscribe(self._on_roas)

    def _on_roas(self, roas):
        # For each roa, subscribe to calibration attribute. Make sure to update acquire button / text if ROC is changed.
        for roa in roas:
            if roa not in self.roa_subscribers:
                roa.roc.subscribe(self._on_va_change)
                self.roa_subscribers.append(roa)
        self._update_roa_count()
        self.check_acquire_button()
        self.update_acquisition_time()  # to update the message

    def _on_va_change(self, _):
        self.check_acquire_button()
        self.update_acquisition_time()  # to update the message

    def check_acquire_button(self):
        self.btn_acquire.Enable(self._main_data_model.is_aligned.value and self.roa_count
                                and not self._get_undefined_calibrations() and
                                not self._main_data_model.is_acquiring.value)  # is_acquiring is True during alignment

    @wxlimit_invocation(1)  # max 1/s
    def update_acquisition_time(self):
        # Update path (in case it's already the next day)
        self.path = datetime.today().strftime('%Y-%m-%d')
        self._tab_panel.txt_destination.SetValue(self.path)

        lvl = None  # icon status shown
        if not self._main_data_model.is_aligned.value:
            lvl = logging.WARN
            txt = "System is not aligned."
        elif self.roa_count == 0:
            lvl = logging.WARN
            txt = "No region of acquisition selected."
        elif self._get_undefined_calibrations():
            lvl = logging.WARN
            txt = "Calibration regions %s missing." % (", ".join(str(c) for c in self._get_undefined_calibrations()),)
        else:
            # Don't update estimated time if acquisition is running (as we are
            # sharing the label with the estimated time-to-completion).
            if self._main_data_model.is_acquiring.value:
                return
            # Display acquisition time
            projects = self._tab_data_model.projects.value
            acq_time = 0
            for p in projects:
                for roa in p.roas.value:
                    acq_time += roa.estimate_acquisition_time()
            acq_time = math.ceil(acq_time)  # round a bit pessimistic
            txt = u"Estimated time is {}."
            txt = txt.format(units.readable_time(acq_time))
        logging.debug("Updating status message %s, with level %s", txt, lvl)
        self.lbl_acqestimate.SetLabel(txt)
        self._show_status_icons(lvl)

    def _get_undefined_calibrations(self):
        """
        returns (list of str): names of ROCs which are undefined
        """
        undefined = set()
        for p in self._tab_data_model.projects.value:
            for roa in p.roas.value:
                roc = roa.roc.value
                if roc.coordinates.value == stream.UNDEFINED_ROI:
                    undefined.add(roc.name.value)
        return sorted(undefined)

    def _update_roa_count(self):
        roas = [roa for p in self._tab_data_model.projects.value for roa in p.roas.value]
        self.txt_num_rois.SetValue("%s" % len(roas))
        self.roa_count = len(roas)

    def _show_status_icons(self, lvl):
        # update status icon to show the logging level
        self.bmp_acq_status_info.Show(lvl in (logging.INFO, logging.DEBUG))
        self.bmp_acq_status_warn.Show(lvl == logging.WARN)
        self._tab_panel.Layout()

    def _reset_acquisition_gui(self, text=None, level=None):
        """
        Set back every GUI elements to be ready for the next acquisition
        text (None or str): a (error) message to display instead of the
          estimated acquisition time
        level (None or logging.*): logging level of the text, shown as an icon.
          If None, no icon is shown.
        """
        self.btn_cancel.Hide()
        self.btn_acquire.Enable()
        self._projectbar_ctrl._project_bar.Enable(True)
        self._calibration_ctrl._calibration_bar.Enable(True)
        self._tab_panel.btn_align.Enable(True)
        self._tab_panel.Layout()

        if text is not None:
            self.lbl_acqestimate.SetLabel(text)
            self._show_status_icons(level)
        else:
            self.update_acquisition_time()

    def on_acquisition(self, evt):
        """
        Start the acquisition (really)
        """
        self._main_data_model.is_acquiring.value = True
        self.btn_acquire.Enable(False)
        self.btn_cancel.Enable(True)
        self.btn_cancel.Show()
        self.gauge_acq.Show()
        self._show_status_icons(None)

        # Don't allow changes to acquisition/calibration ROIs during acquisition
        self._projectbar_ctrl._project_bar.Enable(False)
        self._calibration_ctrl._calibration_bar.Enable(False)
        self._tab_panel.btn_align.Enable(False)

        self.gauge_acq.Range = self.roa_count
        self.gauge_acq.Value = 0

        # Acquire ROAs for all projects
        fs = {}
        for p in self._tab_data_model.projects.value:
            ppath = os.path.join(self.path, p.name.value)  # <acquisition date>/<project name>
            for roa in p.roas.value:
                f = fastem.acquire(roa, ppath, self._main_data_model.ebeam, self._main_data_model.multibeam,
                                   self._main_data_model.descanner, self._main_data_model.mppc,
                                   self._main_data_model.stage, self._main_data_model.ccd,
                                   self._main_data_model.beamshift, self._main_data_model.lens)
                t = roa.estimate_acquisition_time()
                fs[f] = t

        self.acq_future = model.ProgressiveBatchFuture(fs)
        self._fs_connector = ProgressiveFutureConnector(self.acq_future, self.gauge_acq, self.lbl_acqestimate)
        self.acq_future.add_done_callback(self.on_acquisition_done)

    def on_cancel(self, evt):
        """
        Called during acquisition when pressing the cancel button
        """
        fastem._executor.cancel()
        # all the rest will be handled by on_acquisition_done()

    @call_in_wx_main
    def on_acquisition_done(self, future):
        """
        Callback called when the acquisition is finished (either successfully or
        cancelled)
        """
        self.btn_cancel.Hide()
        self.btn_acquire.Enable()
        self.gauge_acq.Hide()
        self._tab_panel.Layout()
        self.lbl_acqestimate.SetLabel("Acquisition done.")
        self._main_data_model.is_acquiring.value = False
        self.acq_future = None
        self._fs_connector = None
        try:
            future.result()
            self._reset_acquisition_gui()
        except CancelledError:
            self._reset_acquisition_gui()
            return
        except Exception as exp:
            # leave the gauge, to give a hint on what went wrong.
            logging.exception("Acquisition failed")
            self._reset_acquisition_gui("Acquisition failed (see log panel).", level=logging.WARNING)
            return


class FastEMCalibrationController:
    """
    Controls the calibration button and process in the calibration panel in the FastEM
    overview and acquisition tab.
    """
    def __init__(self, tab_data, tab_panel, calibrations):
        """
        tab_data: (FastEMAcquisitionGUIData) The representation of the microscope GUI.
        tab_panel: (wx.Frame) The calibration panel, which contains the calibration button,
                    the gauge and the label of the gauge.
        calibrations: (list[Calibrations]) List of calibrations that should be run.
        """
        self._tab_data = tab_data
        self._main_data_model = tab_data.main
        self._panel = tab_panel
        self.calibrations = calibrations

        self.btn_align = tab_panel.btn_align
        self.gauge_progress = tab_panel.align_gauge_progress  # progress bar
        self.gauge_label = tab_panel.align_lbl_gauge  # progress bar label
        self.is_aligned = tab_data.main.is_aligned

        self.gauge_progress.Hide()  # hide progress bar
        self._panel.Parent.Layout()
        self.btn_align.Bind(wx.EVT_BUTTON, self.on_align)

        # check calibration state of system
        # TODO If backend was not restarted, but only GUI, then the system is in principle still calibrated.
        #   However, so far when closing the GUI the system is also not calibrated anymore.
        if not self.is_aligned.value:
            self._reset_calibration_gui()  # display estimated calibration time

        self._future_connector = None  # attribute to store the ProgressiveFutureConnector

    def on_align(self, evt):
        """
        Start or cancel the calibration when the button is triggered.
        :param evt: (GenButtonEvent) Button triggered.
        """
        # check if cancelled
        if self._tab_data.main.is_acquiring.value:
            logging.debug("Calibration was cancelled.")
            align_fastem._executor.cancel()  # all the rest will be handled by on_alignment_done()
            return

        # calibrate
        self._tab_data.main.is_acquiring.value = True  # make sure the acquire/tab buttons are disabled

        self.gauge_progress.Show()  # show progress bar
        # Label is displayed on the right, but during the calibration, we want the gauge bar to occupy the
        # full available space --> use a panel instead of a spacer which can be hidden (spacers cannot) to
        # make it look nice.
        # self._panel.align_lbl_gauge.Hide()  # hide label progress bar TODO put below gauge instead of hide?
        self._panel.align_spacer_panel.Hide()  # hide space progress bar and progress bar label
        self._panel.btn_align.SetLabel("Cancel")  # change button label
        self._panel.Layout()

        # Start alignment
        f = align.fastem.align(self._main_data_model.ebeam, self._main_data_model.multibeam,
                               self._main_data_model.descanner, self._main_data_model.mppc,
                               self._main_data_model.stage, self._main_data_model.ccd,
                               self._main_data_model.beamshift, self._main_data_model.det_rotator,
                               calibrations=self.calibrations)

        f.add_done_callback(self._on_alignment_done)  # also handles cancelling and exceptions
        # connect the future to the progress bar and its label
        self._future_connector = ProgressiveFutureConnector(f, self.gauge_progress, self.gauge_label)

    @call_in_wx_main
    def _on_alignment_done(self, future):
        """
        Called when the calibration is finished (either successfully, cancelled or failed).
        :param future: (ProgressiveFuture) Calibration future object, which can be cancelled.
        """

        self._tab_data.main.is_acquiring.value = False
        self._future_connector = None  # reset connection to the progress bar

        try:
            future.result()  # wait until the calibration is done
            self.is_aligned.value = True  # allow acquiring ROAs
            self._reset_calibration_gui("Calibration successful")
        except CancelledError:
            self.is_aligned.value = False  # don't enable ROA acquisition
            self._reset_calibration_gui("Calibration cancelled")  # update label to indicate cancelling
            return
        except Exception as ex:
            self.is_aligned.value = False  # don't enable ROA acquisition
            logging.exception("Calibration failed with %s.", ex)
            self._reset_calibration_gui("Calibration failed")
            return

    def _reset_calibration_gui(self, text=None):
        """
        Set back every element in the calibration panel to be ready for the next calibration.
        :param text (None or str): A (error) message to display instead of the estimated acquisition time.
        """
        self.btn_align.Enable(True)  # enable button again
        self.btn_align.SetLabel("Calibrate")  # change button label back to ready for calibration
        self.gauge_progress.Hide()  # hide progress bar
        self._panel.align_spacer_panel.Show()  # show space progress bar and progress bar label
        self.gauge_label.Show()  # show label progress bar

        if text is not None:
            self.gauge_label.SetLabel(text)
        else:
            duration = align.fastem.estimate_calibration_time(self.calibrations)
            self.gauge_label.SetLabel(str(duration) + " seconds")

        self._panel.Layout()