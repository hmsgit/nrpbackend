# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END

"""
This module provides helper functions to handle zip
"""

__author__ = 'Hossain Mahmud'

import os
import zipfile
import logging

logger = logging.getLogger(__name__)


def _filter_zip_namelist(zip_namelist):
    """
    Helper function to filter contents of zip. The reason for the filtering
    is that the extractall function breaks if there is a directory inside
    the zip with the same name as the zip, which for some absurd reason
    is treated as a file on exraction and not as a folder.
    :param: zip_namelist: all the folders and files inside a zip
    :return: the list of names without the root directory
    """
    return [name for name in zip_namelist if os.path.dirname(name) is not ""]


class ZipUtil(object):
    """
    This class provides helper functions to handle zip
    """

    @staticmethod
    def extractall(zip_abs_path, extract_to, overwrite=False):  # pragma: no cover
        """
        Extract a zip in the given location
        :param zip_abs_path: absolute path of the zip file
        :param extract_to: absolute path to the folder where to unzip
        :param overwrite: boolean to indicate whether to overwrite the existing contents
        :return: -
        """
        with zipfile.ZipFile(zip_abs_path) as rzip:
            if not overwrite:
                for f in rzip.namelist():
                    if os.path.exists(os.path.join(extract_to, f)):
                        logger.info("Aborting extraction. {file} exists.".format(file=f))
                        return None
            try:
                if not os.path.exists(extract_to):
                    os.makedirs(extract_to)
                with zipfile.ZipFile(zip_abs_path) as rzip:
                    rzip.extractall(extract_to, _filter_zip_namelist(rzip.namelist()))
            except IOError as ex:
                logger.info("Extraction failed due to {err}".format(err=str(ex)))

    @staticmethod
    def create_from_path(path, dest_zip_file):
        """
        Create a zip from a path
        :param path: path to be compressed
        :param dest_zip_file: file name and path to the zip archive
        :return: The created zip file
        """
        with zipfile.ZipFile(dest_zip_file, 'w', zipfile.ZIP_DEFLATED) as zip_f:
            for root, _, files in os.walk(path):
                for f in files:
                    zip_f.write(
                        os.path.join(root, f),
                        os.path.relpath(os.path.join(root, f), os.path.join(path, '..')))

    @staticmethod
    def get_rootname(zip_abs_path):  # pragma: no cover
        """
        Gets the root folder name inside a zip
        Note: for some zip, namelist doesn't contain the folders as entries
        Possibly dependent upon how the zip was bundled, hence the extra check

        :return: root folder name inside a zip
        """
        with zipfile.ZipFile(zip_abs_path) as rzip:
            first_item = rzip.namelist()[0]
            if first_item.endswith('/'):
                return first_item
            elif '/' in first_item:
                return first_item.split('/')[0]
            else:
                return None
