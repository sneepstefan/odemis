# -*- coding: utf-8 -*-
"""
Created on 3 Dec 2012

@author: Éric Piel

Copyright © 2012 Éric Piel, Delmic

This file is part of Odemis.

Odemis is free software: you can redistribute it and/or modify it under the terms
of the GNU General Public License version 2 as published by the Free Software
Foundation.

Odemis is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Odemis. If not, see http://www.gnu.org/licenses/.

"""

from collections.abc import Iterable
import logging
import math
from odemis import model
from odemis.gui.util import call_in_wx_main_wrapper, call_in_wx_main, dead_object_wrapper
from odemis.util import RepeatingTimer
from odemis.util import units
import time
import wx

class VigilantAttributeConnector(object):
    """ This class connects a vigilant attribute with a wxPython control, making sure that the
    changes in one are automatically reflected in the other.

    At the end of the constructor, the value of the VA is assigned to the control.

    Important note: The VA is dominant, meaning the after pausing and resuming, it's always the
    value of the VA that is sent to the control, never the other way around.

    """

    def __init__(self, va, value_ctrl, va_2_ctrl=None, ctrl_2_va=None, events=None):
        """
        va (VigilantAttribute): the VA to connect with
        ctrl (wx.Window): a wx widget to connect to
        va_2_ctrl (None or callable ((value) -> None)): a function to be called
            when the VA is updated, to update the widget. If None, try to use
            the default SetValue(). It is always called in the main WX thread.
            It is called once at initialisation.
        ctrl_2_va (None or callable ((None) -> value)): a function to be called
            when the widget is updated, to update the VA. If None, try to use
            the default GetValue().
            Can raise ValueError, TypeError or IndexError if data is incorrect
        events (None or wx.EVT_* or tuple of wx.EVT_*): events to bind to update
            the value of the VA
        """
        self.vigilattr = va
        self.value_ctrl = value_ctrl
        self.paused = False

        va_2_ctrl = va_2_ctrl or value_ctrl.SetValue
        # Dead_object_wrapper might need/benefit from recognizing bound methods.
        # Or it can be tough to recognize wxPyDeadObjects being passed as 'self'
        self.va_2_ctrl = call_in_wx_main_wrapper(dead_object_wrapper(va_2_ctrl))
        self.ctrl_2_va = ctrl_2_va or value_ctrl.GetValue
        if events is None:
            self.change_events = ()
        elif not isinstance(events, Iterable):
            self.change_events = (events,)
        else:
            self.change_events = events

        if self.value_ctrl:
            self.value_ctrl.Bind(wx.EVT_WINDOW_DESTROY, self._on_ctrl_destroy,
                                 source=self.value_ctrl)

        # Subscribe to the vigilant attribute and initialize
        self._connect(init=True)

    def _on_ctrl_destroy(self, evt):
        self.value_ctrl = None
        self.disconnect()

    def _on_ctrl_value_change(self, evt):
        """ Set the value of the VA when the value of the control is changed

        This handler should only be triggered by events that were generated by direct user actions,
        *not* by programmatically setting the control value!
        """
        # Don't do anything when paused
        if self.paused:
            return

        value = self.ctrl_2_va()
        try:
            logging.debug("Setting VA value to %s after control event %d", value, evt.Id)
            self.vigilattr.value = value
        except (ValueError, TypeError, IndexError) as exc:
            logging.warning("VA refused value %s: %s", value, exc)
            self.va_2_ctrl(self.vigilattr.value)
        evt.Skip()

    def pause(self):
        """ Temporarily prevent VAs from updating controls and controls from updating VAs """
        self.paused = True
        self.vigilattr.unsubscribe(self.va_2_ctrl)

    def resume(self):
        """ Resume updating controls and VAs """
        self.paused = False
        self.vigilattr.subscribe(self.va_2_ctrl, init=True)

    def _connect(self, init):
        logging.debug("Connecting VigilantAttributeConnector")
        self.vigilattr.subscribe(self.va_2_ctrl, init)
        for event in self.change_events:
            self.value_ctrl.Bind(event, self._on_ctrl_value_change)

    def disconnect(self):
        logging.debug("Disconnecting VigilantAttributeConnector")
        if self.value_ctrl:
            for event in self.change_events:
                self.value_ctrl.Unbind(event, handler=self._on_ctrl_value_change)
        self.vigilattr.unsubscribe(self.va_2_ctrl)


class AxisConnector(object):
    """ This class connects the axis of an actuator with a wxPython control,
    making sure that the changes in one are automatically reflected in the
    other.
    """
    def __init__(self, axis, comp, value_ctrl, pos_2_ctrl=None, ctrl_2_pos=None, events=None):
        """
        axis (string): the name of the axis to connect with
        comp (Actuator): the component that contains the axis
        value_ctrl (wx.Window): a wx widget to connect to
        pos_2_ctrl (None or callable ((value) -> None)): a function to be called
            when the position is updated, to update the widget. If None, try to use
            the default SetValue().
        ctrl_2_pos (None or callable ((None) -> value)): a function to be called
            when the widget is updated, to update the VA. If None, try to use
            the default GetValue().
            Can raise ValueError, TypeError or IndexError if data is incorrect
        events (None or wx.EVT_* or tuple of wx.EVT_*): events to bind to update
            the value of the VA
        """
        self.axis = axis
        self.comp = comp
        self.value_ctrl = value_ctrl
        self._prev_focus = None  # the focused control before starting to move
        self._future = None  # Future representing the current move
        pos_2_ctrl = pos_2_ctrl or value_ctrl.SetValue
        self.pos_2_ctrl = call_in_wx_main_wrapper(dead_object_wrapper(pos_2_ctrl))
        self.ctrl_2_pos = ctrl_2_pos or value_ctrl.GetValue
        if events is None:
            self.change_events = ()
        elif not isinstance(events, Iterable):
            self.change_events = (events,)
        else:
            self.change_events = events

        if self.value_ctrl:
            self.value_ctrl.Bind(wx.EVT_WINDOW_DESTROY, self._on_ctrl_destroy,
                                 source=self.value_ctrl)

        # Subscribe to the position and initialize
        self._connect(init=True)

    def _on_ctrl_destroy(self, evt):
        self.value_ctrl = None
        self.disconnect()

    def _on_value_change(self, evt):
        """ This method is called when the value of the control is changed.
        it moves the axis to the new value.
        """
        evt.Skip()

        try:
            if self._future is not None and not self._future.done():
                # Note: when the value is clipped (eg, because the user entered
                # a value too large), COMMAND_ENTER is triggered twice, which
                # is one of the reasons this path is taken.
                logging.info("Not moving axis %s as it's already moving", self.axis)
                return

            value = self.ctrl_2_pos()
            logging.debug("Requesting axis %s to move to %s", self.axis, value)

            # expect absolute move works
            move = {self.axis: value}
            future = self.comp.moveAbs(move)
        except (ValueError, TypeError, IndexError) as exc:
            logging.error("Illegal value: %s", exc)
            future = None

        if future and not future.done():
            # disable the control until the move is finished => gives user
            # feedback and avoids accumulating moves. The drawback is that the
            # GUI focus is lost.
            self._prev_focus = wx.Window.FindFocus()
            self.value_ctrl.Disable()
            self._future = future
            future.add_done_callback(self._on_move_done)
        else:
            # If the move actually did nothing, make sure the text reflects the
            # current position again.
            self._on_pos_change(self.comp.position.value)

    @call_in_wx_main
    def _on_move_done(self, _):
        """ Process the end of the move """
        self._future = None
        # _on_pos_change() is almost always called as well, but not if the move was so small that
        #  the position didn't change. That's why this separate method is needed.
        if self.value_ctrl:
            self.value_ctrl.Enable()
            # Put back the focus on the widget if nothing else got it in the meantime
            if self._prev_focus and wx.Window.FindFocus() is None:
                self._prev_focus.SetFocus()

            # If the move actually did nothing, make sure the text reflects the
            # current position again.
            self._on_pos_change(self.comp.position.value)
        logging.debug("Axis %s finished moving", self.axis)

    def _on_pos_change(self, positions):
        """ Process a position change """
        position = positions[self.axis]
        logging.debug("Axis has moved to position %s", position)
        self.pos_2_ctrl(position)

    def pause(self):
        """ Temporarily prevent position from updating controls """
        self.comp.position.unsubscribe(self._on_pos_change)

    def resume(self):
        """ Resume updating controls """
        self.comp.position.subscribe(self._on_pos_change, init=True)

    def _connect(self, init):
        logging.debug("Connecting AxisConnector")
        self.comp.position.subscribe(self._on_pos_change, init)
        for event in self.change_events:
            self.value_ctrl.Bind(event, self._on_value_change)

    def disconnect(self):
        logging.debug("Disconnecting AxisConnector")
        if self.value_ctrl:
            for event in self.change_events:
                self.value_ctrl.Unbind(event, handler=self._on_value_change)
        self.comp.position.unsubscribe(self._on_pos_change)


PROGRESS_RANGE = 2 ** 12  # Arbitrary large value to always have enough resolution
class ProgressiveFutureConnector(object):
    """ Connects a progressive future to a progress bar and label """

    def __init__(self, future, bar, label=None, full=True):
        """ Update a gauge widget and label, based on the progress reported by the
        ProgressiveFuture.

        future (ProgressiveFuture)
        bar (gauge): the progress bar widget
        label (StaticText or None): if given, will also update a the text with
          the time left.
        full (bool): If True, the time remaining will be displaying in full text
        otherwise a short text will displayed (eg, "1 min and 2 s")
        Note: when the future is complete (done), the progress bar will be set
        to 100%, but the text will not be updated.
        """
        self._future = future
        self._bar = bar
        self._label = label
        self._full_text = full

        # Will contain the info of the future as soon as we get it.
        self._start, self._end = future.get_progress()
        self._end = None
        self._prev_left = None
        self._last_update = 0  # when was the last GUI update

        # a repeating timer, always called in the GUI thread
        self._timer = wx.PyTimer(self._update_progress)
        self._timer.Start(250)  # 4 Hz (250 milliseconds)

        # Set the progress bar to 0
        bar.Range = PROGRESS_RANGE
        bar.Value = 0

        future.add_update_callback(self._on_progress)
        future.add_done_callback(self._on_done)

    def _on_progress(self, _, start, end):
        """ Process any progression

        start (float): time the work started
        end (float): estimated time at which the work is ending

        """

        self._start = start
        self._end = end

    @call_in_wx_main
    def _on_done(self, future):
        """ Process the completion of the future """
        self._timer.Stop()
        if self._future is None:
            logging.warning("Received multiple times completion of future signal")
            return

        if not future.cancelled():
            self._bar.Value = self._bar.Range
            if self._label is None:
                self._bar.SetToolTip("Completed")
        elif self._label is None:
            self._bar.SetToolTip("Cancelled")

        # Drop references to avoid holding the object in memory
        self._future = None
        self._bar = None
        self._label = None

    def _update_progress(self):
        """ Update the progression controls """
        now = time.time()
        past = max(0, now - self._start)
        left = max(0, self._end - now)
        prev_left = self._prev_left
        total = past + left
        if total <= 0:
            logging.warning("Unexpected progress %s to %s", self._start, self._end)
            ratio = 1
        else:
            ratio = past / total

        # Avoid back and forth estimation (but at least every 10 s)
        can_update = True
        if prev_left is not None and self._last_update + 10 > now:
            # Don't update gauge if ratio reduces (a bit)
            try:
                prev_ratio = self._bar.Value / self._bar.Range
                if 1 > prev_ratio / ratio > 1.1:  # decrease < 10 %
                    can_update = False
            except ZeroDivisionError:
                pass
            # Or if the time left in absolute value slightly increases (< 5s)
            if 0 < left - prev_left < 5:
                can_update = False

        if not can_update:
            logging.debug("Not updating progress as new estimation is %g s left "
                          "vs old %g s, and current ratio %g vs old %g.",
                          left, prev_left, ratio * 100, prev_ratio * 100)
            return
        else:
            self._last_update = now

        # Update progress bar
        self._bar.Value = round(PROGRESS_RANGE * ratio)

        if self._future.done():
            # Make sure we don't update the lbl_txt after the future is over as
            # it might be used to display other information
            return

        # Time left text
        self._prev_left = left
        left = math.ceil(left)  # pessimistic

        if left > 2:
            lbl_txt = u"%s" % units.readable_time(left, full=self._full_text)
        else:
            # don't be too precise
            lbl_txt = u"a few seconds"

        if self._full_text:
            lbl_txt += u" left"

        if self._label is None:
            self._bar.SetToolTip(lbl_txt)
        else:
            # TODO: if the text is too big for the label, rewrite with full=False
            # we could try to rely on IsEllipsized() (which requires support for
            # wxST_ELLIPSIZE_END in xrc) or dc.GetTextExtend()
            self._label.SetLabel(lbl_txt)
            self._label.Parent.Layout()


class EllipsisAnimator(RepeatingTimer):
    """ This class animates the special character … (ellipsis) in case it is
        contained in the given message
    """
    def __init__(self, msg, label):
        """
        msg (string): message to be displayed
        label (StaticText): label to be updated with the message text
        """
        super(EllipsisAnimator, self).__init__(0.7, self._updateStatus, "Status update")
        self._status_msg = msg  # Current message to be animated
        self._label = label

    def start(self):
        """ Starts ellipsis animation """
        super(EllipsisAnimator, self).start()
        logging.debug("Will animate message \"%s\"", self._status_msg)
        self._updateStatus()

    @call_in_wx_main
    def _updateStatus(self):
        try:
            # Run in main GUI thread, with some delay on the sleeping thread
            # If was cancelled in between, don't write (old info on) the label
            if self._must_stop.is_set():
                return

            # Compute how many dots to display (0->3)
            n = int((time.time() / self.period) % 4)
            msg = self._status_msg.replace(u"…", u"." * n)
            self._label.SetLabel(msg)
        except Exception:
            logging.exception("Unexpected failure during status message animation")


class ScannerFoVAdapter(object):
    """
    Wrap a stream which has a scanner (as detector) and a .zoom, to behave as a
    "standard" scanner with .horizontalFoV.
    In practice, it's used to control the ScannerSettingsStream via the standard
    zooming functions of a MicroscopeView (fov_hw).
    """

    def __init__(self, setting_stream):
        """
        settings_stream (Stream): stream with .zoom and a .detector
        """
        self._setting_stream = setting_stream
        self._scanner = self._setting_stream.detector
        self.shape = self._scanner.shape
        # For the range, assume the pixelSize will not change
        pxs = self._scanner.pixelSize.value
        zrng = self._setting_stream.zoom.range
        hfov_rng = (self.shape[0] * pxs[0] / zrng[1],
                    self.shape[0] * pxs[0] / zrng[0])
        hfov = self.shape[0] * pxs[0] / self._setting_stream.zoom.value
        self.horizontalFoV = model.FloatContinuous(hfov, range=hfov_rng, unit="m",
                                                   setter=self._set_hfov)
        self.horizontalFoV.clip_on_range = True

        self._scanner.pixelSize.subscribe(self._on_pixel_size)
        self._setting_stream.zoom.subscribe(self._update_hfov)

    def _set_hfov(self, hfov):
        pxs = self._scanner.pixelSize.value
        zoom = self.shape[0] * pxs[0] / hfov

        # Unsubscribe temporarily to avoid any chance of infinite recursion
        self._setting_stream.zoom.unsubscribe(self._update_hfov)
        self._setting_stream.zoom.value = self._setting_stream.zoom.clip(zoom)
        self._setting_stream.zoom.subscribe(self._update_hfov)

        zoom = self._setting_stream.zoom.value
        hfov = self.shape[0] * pxs[0] / zoom
        logging.debug("Updating zoom to %s => HFW %s m", zoom, hfov)
        return hfov

    def _on_pixel_size(self, pxs):
        """
        Called when the lens magnification is changed (ie, rarely)
        """
        # All the FoV values need to be updated
        zrng = self._setting_stream.zoom.range
        hfov_rng = (self.shape[0] * pxs[0] / zrng[1],
                    self.shape[0] * pxs[0] / zrng[0])
        self.horizontalFoV.range = hfov_rng
        self._update_hfov()

    def _update_hfov(self, _=None):
        zoom = self._setting_stream.zoom.value
        pxs = self._scanner.pixelSize.value
        hfov = self.shape[0] * pxs[0] / zoom
        self.horizontalFoV.value = hfov

