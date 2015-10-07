#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 5 Jan 2015

@author: Kimon Tsitsikas

Copyright © 2014-2015 Kimon Tsitsikas, Delmic

This file is part of Odemis.

Odemis is free software: you can redistribute it and/or modify it under the
terms  of the GNU General Public License version 2 as published by the Free
Software  Foundation.

Odemis is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY;  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR  PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
Odemis. If not, see http://www.gnu.org/licenses/.
"""

# This script allows the user to perform the whole delphi calibration procedure
# step by step in a semi-manual way. It attempts to apply each step automatically
# and in case of failure it waits for the user to perform the step failed manually.
#
# run as:
# python delphi_man_calib.py
#
# You first need to run the odemis backend with the DELPHI config:
# odemisd --log-level 2 install/linux/usr/share/odemis/delphi.odm.yaml



from __future__ import division

import logging
from odemis import model
import odemis.acq.align.delphi as aligndelphi
import sys
from odemis.acq import align
from odemis.gui.conf import get_calib_conf
import math
from odemis.acq.align import autofocus

logging.getLogger().setLevel(logging.WARNING)


def main(args):
    """
    Handles the command line arguments
    args is the list of arguments passed
    return (int): value to return to the OS as program exit code
    """
    try:
        escan = None
        detector = None
        ccd = None
        # find components by their role
        for c in model.getComponents():
            if c.role == "e-beam":
                escan = c
            elif c.role == "bs-detector":
                detector = c
            elif c.role == "ccd":
                ccd = c
            elif c.role == "sem-stage":
                sem_stage = c
            elif c.role == "align":
                opt_stage = c
            elif c.role == "ebeam-focus":
                ebeam_focus = c
            elif c.role == "overview-focus":
                navcam_focus = c
            elif c.role == "focus":
                focus = c
            elif c.role == "overview-ccd":
                overview_ccd = c
            elif c.role == "chamber":
                chamber = c
        if not all([escan, detector, ccd]):
            logging.error("Failed to find all the components")
            raise KeyError("Not all components found")

        # Get pressure values
        pressures = chamber.axes["pressure"].choices
        vacuum_pressure = min(pressures.keys())
        vented_pressure = max(pressures.keys())
        if overview_ccd:
            for p, pn in pressures.items():
                if pn == "overview":
                    overview_pressure = p
                    break
            else:
                overview_pressure = None

        calibconf = get_calib_conf()
        shid, sht = chamber.sampleHolder.value
        calib_values = calibconf.get_sh_calib(shid)
        if calib_values is None:
            first_hole = second_hole = hole_focus = offset = scaling = rotation = iscale = irot = iscale_xy = ishear = resa = resb = hfwa = spotshift = None
        else:
            first_hole, second_hole, hole_focus, offset, scaling, rotation, iscale, irot, iscale_xy, ishear, resa, resb, hfwa, spotshift = calib_values
        print '\033[1;36m'
        print "**Delphi Manual Calibration steps**"
        print "1.Sample holder hole detection"
        print "    Current values: 1st hole: " + str(first_hole)
        print "                    2st hole: " + str(second_hole)
        print "                    hole focus: " + str(hole_focus)
        print "2.Twin stage calibration"
        print "    Current values: offset: " + str(offset)
        print "                    scaling: " + str(scaling)
        print "                    rotation: " + str(rotation)
        print "3.SEM image calibration"
        print "    Current values: resolution-a: " + str(resa)
        print "                    resolution-b: " + str(resb)
        print "                    hfw-a: " + str(hfwa)
        print "                    spot shift: " + str(spotshift)
        print "4.Fine alignment"
        print "    Current values: scale: " + str(iscale)
        print "                    rotation: " + str(irot)
        print "                    scale-xy: " + str(iscale_xy)
        print "                    shear: " + str(ishear)
        print '\033[1;m'
        print "\033[1;31mNote that you should not perform any stage move during the process. \nInstead, you may zoom in/out while focusing.\033[1;m"
        print "\033[1;30mNow initializing, please wait...\033[1;m"

        # Move to the overview position first
        f = chamber.moveAbs({"pressure": overview_pressure})
        f.result()

        # Reference the (optical) stage
        f = opt_stage.reference({"x", "y"})
        f.result()

        f = focus.reference({"z"})
        f.result()

        # SEM stage to (0,0)
        f = sem_stage.moveAbs({"x": 0, "y": 0})
        f.result()

        # Calculate offset approximation
        try:
            f = aligndelphi.LensAlignment(overview_ccd, sem_stage)
            position = f.result()
        except Exception:
            raise IOError("Lens alignment failed.")

        # Just to check if move makes sense
        f = sem_stage.moveAbs({"x": position[0], "y": position[1]})
        f.result()

        # Move to SEM
        f = chamber.moveAbs({"pressure": vacuum_pressure})
        f.result()

        while True:
            ans = None
            while (ans not in ["y", "n", "Y", "N", '']):
                msg = "\033[1;35mDo you want to execute the sample holder hole detection? [Y/n]\033[1;m"
                ans = raw_input(msg)
            if ans in ["Y", "y", '']:
                # Compute stage calibration values
                # Detect the holes/markers of the sample holder
                # Move Phenom sample stage to expected hole position
                f = sem_stage.moveAbs(aligndelphi.EXPECTED_HOLES[0])
                f.result()
                # Set the FoV to almost 2mm
                escan.horizontalFoV.value = escan.horizontalFoV.range[1]
                msg = "\033[1;34mPlease turn on the SEM stream and focus the SEM image. Then turn off the stream and press Enter ...\033[1;m"
                raw_input(msg)
                print "\033[1;30mTrying to detect the holes/markers, please wait...\033[1;m"
                try:
                    hole_detectionf = aligndelphi.HoleDetection(detector, escan, sem_stage,
                                                                ebeam_focus, known_focus=None, manual=True)
                    new_first_hole, new_second_hole, new_hole_focus = hole_detectionf.result()
                    new_hole_focus = ebeam_focus.position.value.get('z')
                    print '\033[1;35m'
                    print "Values computed: 1st hole: " + str(new_first_hole)
                    print "                 2st hole: " + str(new_second_hole)
                    print "                 hole focus: " + str(new_hole_focus)
                    print '\033[1;m'
                    ans = None
                    while (ans not in ["y", "n", "Y", "N", '']):
                        msg = "\033[1;35mDo you want to update the calibration file with these values? [Y/n]\033[1;m"
                        ans = raw_input(msg)
                    if ans in ["Y", "y", '']:
                        first_hole, second_hole, hole_focus = new_first_hole, new_second_hole, new_hole_focus
                        calibconf.set_sh_calib(shid, first_hole, second_hole, hole_focus, offset,
                               scaling, rotation, iscale, irot, iscale_xy, ishear,
                               resa, resb, hfwa, spotshift)
                    break
                except IOError:
                    print "\033[1;31mSample holder hole detection failed.\033[1;m"
            else:
                break

        f = sem_stage.moveAbs({"x":position[0], "y":position[1]})
        f.result()

        f = opt_stage.moveAbs({"x": 0, "y": 0})
        f.result()
        # Set min fov
        # We want to be as close as possible to the center when we are zoomed in
        escan.horizontalFoV.value = escan.horizontalFoV.range[0]
        pure_offset = None
        center_focus = None

        while True:
            ans = None
            while (ans not in ["y", "n", "Y", "N", '']):
                msg = "\033[1;35mDo you want to execute the twin stage calibration? [Y/n]\033[1;m"
                ans = raw_input(msg)
            if ans in ["Y", "y", '']:
                # Configure CCD and e-beam to write CL spots
                ccd.binning.value = (1, 1)
                ccd.resolution.value = ccd.resolution.range[1]
                ccd.exposureTime.value = 900e-03
                escan.scale.value = (1, 1)
                escan.resolution.value = (1, 1)
                escan.translation.value = (0, 0)
                escan.shift.value = (0, 0)
                escan.dwellTime.value = 5e-06
                detector.data.subscribe(_discard_data)
                msg = "\033[1;34mPlease turn on the Optical stream, set Power to 0 Watt and focus the image so you have a clearly visible spot. Then turn off the stream and press Enter...\033[1;m"
                raw_input(msg)
                print "\033[1;30mCalculating translation, please wait...\033[1;m"
                detector.data.unsubscribe(_discard_data)
                try:
                    align_offsetf = aligndelphi.AlignAndOffset(ccd, detector, escan, sem_stage,
                                                               opt_stage, focus)
                    align_offset = align_offsetf.result()
                    center_focus = focus.position.value.get('z')

                    print "\033[1;30mCalculating rotation and scaling, please wait...\033[1;m"
                    rotation_scalingf = aligndelphi.RotationAndScaling(ccd, detector, escan, sem_stage,
                                                                       opt_stage, focus, align_offset, manual=True)
                    acc_offset, new_rotation, new_scaling = rotation_scalingf.result()

                    # Offset is divided by scaling, since Convert Stage applies scaling
                    # also in the given offset
                    pure_offset = acc_offset
                    new_offset = ((acc_offset[0] / new_scaling[0]), (acc_offset[1] / new_scaling[1]))

                    print '\033[1;35m'
                    print "Values computed: offset: " + str(new_offset)
                    print "                 scaling: " + str(new_scaling)
                    print "                 rotation: " + str(new_rotation)
                    print '\033[1;m'
                    ans = None
                    while (ans not in ["y", "n", "Y", "N", '']):
                        msg = "\033[1;35mDo you want to update the calibration file with these values? [Y/n]\033[1;m"
                        ans = raw_input(msg)
                    if ans in ["Y", "y", '']:
                        offset, scaling, rotation = new_offset, new_scaling, new_rotation
                        calibconf.set_sh_calib(shid, first_hole, second_hole, hole_focus, offset,
                               scaling, rotation, iscale, irot, iscale_xy, ishear,
                               resa, resb, hfwa, spotshift)
                    break
                except IOError:
                    print "\033[1;31mTwin stage calibration failed.\033[1;m"
            else:
                break

        while True:
            ans = None
            while (ans not in ["y", "n", "Y", "N", '']):
                msg = "\033[1;35mDo you want to execute the SEM image calibration? [Y/n]\033[1;m"
                ans = raw_input(msg)
            if ans in ["Y", "y", '']:
                f = opt_stage.moveAbs({"x": 0, "y": 0})
                f.result()
                if pure_offset is not None:
                    f = sem_stage.moveAbs({"x":pure_offset[0], "y":pure_offset[1]})
                    f.result()
                elif offset is not None:
                    f = sem_stage.moveAbs({"x":offset[0] * scaling[0], "y":offset[1] * scaling[1]})
                    f.result()
                else:
                    f = sem_stage.moveAbs({"x":position[0], "y":position[1]})
                    f.result()
                if center_focus is not None:
                    f = focus.moveAbs({"z": center_focus})
                    f.result()
                try:
                    # Compute spot shift percentage
                    # Configure CCD and e-beam to write CL spots
                    ccd.binning.value = (1, 1)
                    ccd.resolution.value = ccd.resolution.range[1]
                    ccd.exposureTime.value = 900e-03
                    escan.scale.value = (1, 1)
                    escan.resolution.value = (1, 1)
                    escan.translation.value = (0, 0)
                    escan.shift.value = (0, 0)
                    escan.dwellTime.value = 5e-06
                    detector.data.subscribe(_discard_data)
                    msg = "\033[1;34mPlease turn on the Optical stream, set Power to 0 Watt and focus the image so you have a clearly visible spot. Then turn off the stream and press Enter ...\033[1;m"
                    raw_input(msg)
                    print "\033[1;30mCalculating spot shift, please wait...\033[1;m"
                    detector.data.unsubscribe(_discard_data)
                    spot_shiftf = aligndelphi.SpotShiftFactor(ccd, detector, escan, focus)
                    new_spotshift = spot_shiftf.result()

                    print "\033[1;30mCalculating resolution and HFW shift, please wait...\033[1;m"
                    # Compute resolution-related values
                    resolution_shiftf = aligndelphi.ResolutionShiftFactor(detector, escan, sem_stage, ebeam_focus, hole_focus)
                    new_resa, new_resb = resolution_shiftf.result()

                    # Compute HFW-related values
                    hfw_shiftf = aligndelphi.HFWShiftFactor(detector, escan, sem_stage, ebeam_focus, hole_focus)
                    new_hfwa = hfw_shiftf.result()

                    print '\033[1;35m'
                    print "Values computed: resolution-a: " + str(new_resa)
                    print "                 resolution-b: " + str(new_resb)
                    print "                 hfw-a: " + str(new_hfwa)
                    print "                 spot shift: " + str(new_spotshift)
                    print '\033[1;m'
                    ans = None
                    while (ans not in ["y", "n", "Y", "N", '']):
                        msg = "\033[1;35mDo you want to update the calibration file with these values? [Y/n]\033[1;m"
                        ans = raw_input(msg)
                    if ans in ["Y", "y", '']:
                        resa, resb, hfwa, spotshift = new_resa, new_resb, new_hfwa, new_spotshift
                        calibconf.set_sh_calib(shid, first_hole, second_hole, hole_focus, offset,
                               scaling, rotation, iscale, irot, iscale_xy, ishear,
                               resa, resb, hfwa, spotshift)
                    break
                except IOError:
                    print "\033[1;31mSEM image calibration failed.\033[1;m"
            else:
                break

        while True:
            ans = None
            while (ans not in ["y", "n", "Y", "N", '']):
                msg = "\033[1;35mDo you want to execute the fine alignment? [Y/n]\033[1;m"
                ans = raw_input(msg)
            if ans in ["Y", "y", '']:
                # Return to the center so fine alignment can be executed just after calibration
                f = opt_stage.moveAbs({"x": 0, "y": 0})
                f.result()
                if pure_offset is not None:
                    f = sem_stage.moveAbs({"x":pure_offset[0], "y":pure_offset[1]})
                    f.result()
                elif offset is not None:
                    f = sem_stage.moveAbs({"x":offset[0] * scaling[0], "y":offset[1] * scaling[1]})
                    f.result()
                else:
                    f = sem_stage.moveAbs({"x":position[0], "y":position[1]})
                    f.result()
                if center_focus is not None:
                    f = focus.moveAbs({"z": center_focus})
                    f.result()

                # Focus the CL spot using SEM focus
                # Configure CCD and e-beam to write CL spots
                ccd.binning.value = (1, 1)
                ccd.resolution.value = ccd.resolution.range[1]
                ccd.exposureTime.value = 900e-03
                escan.horizontalFoV.value = escan.horizontalFoV.range[0]
                escan.scale.value = (1, 1)
                escan.resolution.value = (1, 1)
                escan.translation.value = (0, 0)
                escan.shift.value = (0, 0)
                escan.dwellTime.value = 5e-06
                det_dataflow = detector.data

                # Refocus the SEM
                escan.resolution.value = (512, 512)
                msg = "\033[1;34mPlease turn on the SEM stream and focus the SEM image if needed. Then turn off the stream and press Enter ...\033[1;m"
                raw_input(msg)

                # Run the optical fine alignment
                # TODO: reuse the exposure time
                # Configure CCD and e-beam to write CL spots
                ccd.binning.value = (1, 1)
                ccd.resolution.value = ccd.resolution.range[1]
                ccd.exposureTime.value = 900e-03
                escan.scale.value = (1, 1)
                escan.resolution.value = (1, 1)
                escan.translation.value = (0, 0)
                escan.shift.value = (0, 0)
                escan.dwellTime.value = 5e-06
                detector.data.subscribe(_discard_data)
                msg = "\033[1;34mPlease turn on the Optical stream, set Power to 0 Watt and focus the image so you have a clearly visible spot. Then turn off the stream and press Enter...\033[1;m"
                raw_input(msg)
                print "\033[1;30mFine alignment in progress, please wait...\033[1;m"
                detector.data.unsubscribe(_discard_data)
                try:
                    escan.horizontalFoV.value = 80e-06
                    f = align.FindOverlay((4, 4),
                                          0.5,  # s, dwell time
                                          10e-06,  # m, maximum difference allowed
                                          escan,
                                          ccd,
                                          detector,
                                          skew=True,
                                          bgsub=True)
                    trans_val, cor_md = f.result()
                    trans_md, skew_md = cor_md
                    new_iscale = trans_md[model.MD_PIXEL_SIZE_COR]
                    new_irot = -trans_md[model.MD_ROTATION_COR] % (2 * math.pi)
                    new_ishear = skew_md[model.MD_SHEAR_COR]
                    new_iscale_xy = skew_md[model.MD_PIXEL_SIZE_COR]
                    print '\033[1;35m'
                    print "Values computed: scale: " + str(new_iscale)
                    print "                 rotation: " + str(new_irot)
                    print "                 scale-xy: " + str(new_iscale_xy)
                    print "                 shear: " + str(new_ishear)
                    print '\033[1;m'
                    ans = None
                    while (ans not in ["y", "n", "Y", "N", '']):
                        msg = "\033[1;35mDo you want to update the calibration file with these values? [Y/n]\033[1;m"
                        ans = raw_input(msg)
                    if ans in ["Y", "y", '']:
                        iscale, irot, iscale_xy, ishear = new_iscale, new_irot, new_iscale_xy, new_ishear
                        calibconf.set_sh_calib(shid, first_hole, second_hole, hole_focus, offset,
                               scaling, rotation, iscale, irot, iscale_xy, ishear,
                               resa, resb, hfwa, spotshift)
                    break
                except IOError:
                    print "\033[1;31mSEM image calibration failed.\033[1;m"
            else:
                break

        # Update calibration file
        print "\033[1;30mUpdating calibration file is done, now ejecting, please wait...\033[1;m"
    except KeyboardInterrupt:
        logging.warning("Manual calibration procedure was cancelled.")
    except:
        logging.exception("Unexpected error while performing action.")
        return 127
    finally:
        # Eject the sample holder
        f = chamber.moveAbs({"pressure": vented_pressure})
        f.result()

    return 0


def _discard_data(df, data):
    """
    Does nothing, just discard the SEM data received (for spot mode)
    """
    pass

if __name__ == '__main__':
    ret = main(sys.argv)
    logging.shutdown()
    exit(ret)
