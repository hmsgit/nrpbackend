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
This module provides support methods to perform I/O operation of exc and bibi files.
"""
# TODO: re-implement is class as a helper to SimulationConfig (yet to be introduced)
import os
import re
import logging

from hbp_nrp_commons.generated import exp_conf_api_gen as excXmlParser
from hbp_nrp_commons.generated import bibi_api_gen as bibiXmlParser

logger = logging.getLogger(__name__)


# This class is being intentionally excluded from code coverage
class ExcBibiHandler(object):   # pragma: no cover
    """
    Helper class for ROSCLEServer to handle read write operations on exc and bibi in Storage
    """

    def __init__(self, assembly):
        self._cle_assembly = assembly

    def add_robotpose(self, robot_id, pose=None):
        """
        Adds a <robotPose> tag in the exc

        :param robot_id: robotId attribute for the tag
        :param pose: A cle_ros_msgs.msgs.Pose object (defines an object's Euler pos and orientation)
        :return:
        """
        with open(self._cle_assembly.exc.path, 'r') as excFile:
            exc = excFile.read()

        # Get tag prefix if there's one in the tag, e.g., <ns1:ExD>
        tag = ''
        match = re.search(r'([\w\d]*:)ExD', exc)
        if match:
            tag = match.group(0)[:-3]

        robotpose = '<{tag}robotPose robotId="{id}" x="{x}" y="{y}" z="{z}" ' \
                    'roll="{r}" pitch="{p}" yaw="{q}" />\n' \
            .format(id=robot_id,
                    x=pose.x if pose else '0.0',
                    y=pose.y if pose else '0.0',
                    z=pose.z if pose else '0.0',
                    r=pose.roll if pose else '0.0',
                    p=pose.pitch if pose else '0.0',
                    q=pose.yaw if pose else '0.0',
                    tag=tag)

        logger.info("Adding " + str(robotpose) + " to exc")
        # Replace </environmentModel> with <robotPose ... .../></environmentModel>
        exc = re.sub(r'<\/([\w\d]*:*)environmentModel>', robotpose + r'</\g<1>environmentModel>',
                     exc, 1)

        # Update DOM object, don't know any better way. Probably not been used anyway
        path = self._cle_assembly.exc.path
        self._cle_assembly.exc = excXmlParser.CreateFromDocument(exc)
        self._cle_assembly.exc.path = path
        self._cle_assembly.exc.dir = os.path.dirname(path)

        # Update sim dir copy of the exc
        self.rewrite_exc(exc)

        # update storage's copy
        self._cle_assembly.storage_client.create_or_update(
            self._cle_assembly.token,
            self._cle_assembly.experiment_id,
            os.path.basename(self._cle_assembly.exc.path),
            self._prettify_xml(exc),
            "text/plain"
        )

    def delete_robotpose(self, robot_id):
        """
        Edit <robotPose> tag in the exc where robotId=robot_id

        :param robot_id: robotId attribute for the tag
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """
        with open(self._cle_assembly.exc.path, 'r') as excFile:
            exc = excFile.read()

        # trial and errored. try https://regexr.com/
        # special case for robot with id 'robot'
        # TODO: remove special handling of id 'robot' once we make robotId a must
        if robot_id == 'robot':
            regex = r'>[\s]*<[\w\d]*:*robotPose[^>]*>'
        else:
            regex = r'>[\s]*<[\w\d]*:*robotPose[^>]*robotId *= *[\'"]' + robot_id + r'[\'"][^>]*\/>'

        logger.info("Removing robotPose tag for {0} from exc".format(robot_id))
        exc = re.sub(regex, '>', exc, 1)

        # Update DOM object, don't know any better way. Probably not been used anyway
        path = self._cle_assembly.exc.path
        self._cle_assembly.exc = excXmlParser.CreateFromDocument(exc)
        self._cle_assembly.exc.path = path
        self._cle_assembly.exc.dir = os.path.dirname(path)

        # Update sim dir copy of the exc
        self.rewrite_exc(exc)

        # update storage's copy
        self._cle_assembly.storage_client.create_or_update(
            self._cle_assembly.token,
            self._cle_assembly.experiment_id,
            os.path.basename(self._cle_assembly.exc.path),
            self._prettify_xml(exc),
            "text/plain"
        )

    def update_robotpose(self, robot_id, pose):
        """
        Edit <robotPose> tag in the exc where robotId=robot_id

        :param robot_id: robotId attribute for the tag
        :param pose: A cle_ros_msgs.msgs.Pose object (defines an object's Euler pos and orientation)
        :return: Tuple (True, SDF relative path) or (False, error message) to update config files
        """
        if not robot_id in self._cle_assembly.robotManager.get_robot_dict():
            return False, "No robot exists with id equals {id}".format(id=robot_id)
        try:
            self.delete_robotpose(robot_id)
            self.add_robotpose(robot_id, pose)
        # pylint: disable=broad-except
        except Exception as e:
            return False, "An error occurred while updating robotPose {err}".format(err=e)
        return True, "Tag updated successfully"

    def add_bodymodel(self, robot_id, model_path, is_custom, zip_path=None):
        """
        Adds a <bodyModel> tag in the bibi

        :param robot_id: attribute robotId in the tag
        :param model_path: value() fo the tag
        :param is_custom: attribute customAsset in the tag
        :param zip_path: if custom, then value of the assetPath attribute
        :return:
        """
        if is_custom and zip_path is None:
            raise Exception("Please provide the custom zip location")

        with open(self._cle_assembly.bibi.path, 'r') as bibiFile:
            bibi = bibiFile.read()

        # Get tag prefix if there's one in the tag, e.g., <ns1:ExD>
        tag = ''
        match = re.search(r'([\w\d]*:bibi)', bibi)
        if match:
            tag = match.group(0)[:-4]

        bodymodel = '<{tag}bodyModel robotId="{id}" customAsset="{customAsset}" ' \
                    '{assetPath}>{value}</{tag}bodyModel>' \
            .format(id=robot_id,
                    customAsset="true" if is_custom else "false",
                    assetPath='assetPath="' + zip_path + '"' if is_custom else '',
                    value=model_path,
                    tag=tag)

        logger.info("Adding " + str(bodymodel) + " to bibi")

        # Replace </bodyModel> with </bodyModel><bodyModel ... ... </bodyModel>
        # If there's no robot add the tag before the ending </bibi> tag
        if 'bodyModel' in bibi:
            bibi = re.sub(r'<\/(.*)bodyModel>', r'</\g<1>bodyModel>' + bodymodel, bibi, 1)
        else:
            if re.search(r'<(.*)bibi([^<>]*)/>', bibi):
                # basically empty <bibi />
                bibi = re.sub(r'<(.*)bibi([^<>]*)/>',
                              r'<\g<1>bibi\g<2>>' + bodymodel + r'</\g<1>bibi>',
                              bibi, 1)
            else:
                bibi = re.sub(r'<\/(.*)bibi>', bodymodel + r'</\g<1>bibi>', bibi, 1)

        # Update DOM object, don't know any better way
        path = self._cle_assembly.bibi.path
        self._cle_assembly.bibi = bibiXmlParser.CreateFromDocument(bibi)
        self._cle_assembly.bibi.path = path
        self._cle_assembly.bibi.dir = os.path.dirname(path)

        # Update sim dir copy of the bibi
        self.rewrite_bibi(bibi)

        # update storage's copy
        self._cle_assembly.storage_client.create_or_update(
            self._cle_assembly.token,
            self._cle_assembly.experiment_id,
            self._cle_assembly.exc.bibiConf.src,
            self._prettify_xml(bibi),
            "text/plain"
        )

    def delete_bodymodel(self, robot_id):
        """
        Deletes a <bodyModel> tag from the bibi

        :param robot_id: attribute robotId in the tag
        :return:
        """
        with open(self._cle_assembly.bibi.path, 'r') as bibiFile:
            bibi = bibiFile.read()

        # trial and errored. try https://regexr.com/
        # special case for robot with id 'robot'
        # TODO: remove special handling of id 'robot' once we make robotId a must
        if robot_id == 'robot':
            regex = r'>[\s]*<[\w\d]*:*bodyModel.*>'
        else:
            regex = r'>[\s]*<[\w\d]*:*bodyModel[^>]*robotId *= *[\'"]' \
                    + robot_id + r'[\'"][^<]*>' +\
                    r'[\w\d/\.]*</[\w\d]*:*bodyModel>'

        logger.info("Removing bodyModel tag for {0} from bibi".format(robot_id))
        bibi = re.sub(regex, '>', bibi, 1)

        # Update DOM object, don't know any better way
        path = self._cle_assembly.bibi.path
        self._cle_assembly.bibi = bibiXmlParser.CreateFromDocument(bibi)
        self._cle_assembly.bibi.path = path
        self._cle_assembly.bibi.dir = os.path.dirname(path)

        # Update sim dir copy of the bibi
        self.rewrite_bibi(bibi)

        # update storage's copy
        self._cle_assembly.storage_client.create_or_update(
            self._cle_assembly.token,
            self._cle_assembly.experiment_id,
            self._cle_assembly.exc.bibiConf.src,
            self._prettify_xml(bibi),
            "text/plain"
        )

    def _prettify_xml(self, plain_text):
        """
        Format a given xml text

        :param plain_text: xml text string
        :return: formatted xml string
        """
        # pylint: disable=no-self-use
        import lxml
        return lxml.etree.tostring(lxml.etree.XML(plain_text), pretty_print=True)

    def _write_xml(self, plain_text, filename):
        """
        Write xml into file. If the file exists, it overwrites the content.

        :param plain_text: xml text
        :param filename: absolute path to the file to write
        :return: Tuple (True, None) or (False, error)
        """
        # pylint: disable=no-self-use, broad-except
        try:
            with open(filename, 'w') as f:
                try:
                    f.write(self._prettify_xml(plain_text))
                except IOError as e:
                    return False, str(e)
        except Exception as e:
            return False, str(e)

        return True, None

    def rewrite_exc(self, plain_xml):
        """
        Rewrites exc file in the simulation directory

        :param plain_xml: xml text
        :return: -
        """
        # pylint: disable=no-self-use
        return self._write_xml(
            plain_xml,
            self._cle_assembly.exc.path
        )

    def rewrite_bibi(self, plain_xml):
        """
        Rewrites bibi file in the simulation directory

        :param plain_xml: xml text
        :return: -
        """
        # pylint: disable=no-self-use
        return self._write_xml(
            plain_xml,
            self._cle_assembly.bibi.path
        )
