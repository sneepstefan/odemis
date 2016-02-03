# -*- coding: utf-8 -*-
"""
:author: Rinze de Laat <laat@delmic.com>
:copyright: © 2015 Rinze de Laat, Delmic

This file is part of Odemis.

.. license::
    Odemis is free software: you can redistribute it and/or modify it under the terms of the GNU
    General Public License version 2 as published by the Free Software Foundation.

    Odemis is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License along with Odemis. If not,
    see http://www.gnu.org/licenses/.

This module contains update functionality for the Windows Viewer version of Odemis

"""
from __future__ import division

import logging
import odemis
from odemis.gui.conf import get_general_conf
import os
import pkg_resources
import subprocess
import tempfile
import urllib2
import wx


VERSION_FILE = "version.txt"
INSTALLER_FILE = "OdemisViewer-%s.exe"
VIEWER_NAME = "Odemis Viewer"
VIEWER_ROOT_URL = "http://www.delmic.com/odemisviewer/"


class WindowsUpdater:
    def __init__(self):
        try:
            if wx.GetApp()._is_standalone == "delphi":
                global INSTALLER_FILE
                global VIEWER_NAME
                INSTALLER_FILE = "DelphiViewer-%s.exe"
                VIEWER_NAME = "Delphi Viewer"
        except Exception:
            logging.info("Considering the app as a standard Odemis", exc_info=True)

    @staticmethod
    def get_local_version():
        """ Get the local version of Odemis
        return (str): version of the form #.#.##
         """

        ver_str = odemis._get_version()
        if '-' in ver_str:
            ver_str = '.'.join(ver_str.split('-')[:2])
        return ver_str

    @staticmethod
    def get_remote_version():
        """ Get the remote version of Odemis as a string

        return (None or str, int): version of the form #.#.##, size of the installer to install (in
        bytes)

        """

        web_version = None
        web_size = 0

        try:
            web_version_file = urllib2.urlopen(os.path.join(VIEWER_ROOT_URL, VERSION_FILE),
                                               timeout=10)
            web_version = web_version_file.readline().strip()
            web_size = int(web_version_file.readline().strip())
            web_version_file.close()
        except IOError as err:
            logging.warn("Error on remote version check (%s)" % err)

        return web_version, web_size

    def check_for_update(self):
        """ Check if a newer version is available online and offer to update """
        # TODO: just return True or False, and let the caller call show_update_dialog()
        logging.info("Retrieving version info...")

        web_version, web_size = self.get_remote_version()

        if web_version is None:
            logging.info("Could not retrieve remote version, will not update")
            return

        logging.info("Found remote version %s", web_version)

        lv = pkg_resources.parse_version(self.get_local_version())
        rv = pkg_resources.parse_version(web_version)
        if rv <= lv:
            return

        logging.info("Newer version found, suggesting update...")

        self.show_update_dialog(web_version, web_size)

    def show_update_dialog(self, web_version, web_size):
        """ Show update dialog

        Args:
            web_version: (str) Version of the installer on the website
            web_size: (str) The byte size of the installer
        """

        answer = wx.MessageBox(
            'Version %s of %s is available.\n\nDo you want to update?' % (web_version, VIEWER_NAME),
            "New version available", wx.YES_NO | wx.ICON_INFORMATION
        )

        if answer == wx.YES:
            self.download_installer(web_version, web_size)
        else:
            answer = wx.MessageBox(
                "Would you like to be notified of future updates to %s?" % VIEWER_NAME,
                "Update checker", wx.YES_NO | wx.ICON_QUESTION
            )

            if answer == wx.NO:
                conf = get_general_conf()
                conf.set("viewer", "update", "no")

    def download_installer(self, remote_version, web_size):

        pdlg = None

        try:
            dest_dir = tempfile.gettempdir()

            installer_file = INSTALLER_FILE % remote_version

            web_url = os.path.join(VIEWER_ROOT_URL, installer_file)
            web_file = urllib2.urlopen(web_url, timeout=10)

            local_path = os.path.join(dest_dir, installer_file)
            local_file = open(local_path, 'wb')

            maximum = web_size

            logging.info("Downloading from %s to %s...", web_url, local_path)

            pdlg = wx.ProgressDialog(
                "Downloading update...",
                "The new %s installer is being downloaded." % VIEWER_NAME,
                maximum=maximum,
                parent=wx.GetApp().main_frame,
                style=wx.PD_CAN_ABORT | wx.PD_APP_MODAL | wx.PD_ELAPSED_TIME)

            keep_going = True
            count = 0

            while keep_going and count < maximum:
                grabbed = web_file.read(4096)
                local_file.write(grabbed)
                if grabbed == "":
                    count = maximum
                else:
                    count += 4096
                (keep_going, skip) = pdlg.Update(count)

            pdlg.Destroy()

            web_file.close()
            local_file.close()
            logging.info("Done.")

            if keep_going:
                self.run_installer(local_path)
        except Exception:
            logging.exception("Failure to download!")
            try:
                pdlg.Destroy()
            except (wx.PyDeadObjectError, AttributeError):
                pass

    @staticmethod
    def run_installer(local_path):
        try:
            subprocess.call(local_path)
        except WindowsError, (err_nr, _):
            if err_nr == 740:
                os.startfile(local_path, "runas")
