# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
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
import sensor_msgs.msg


@nrp.MapRetina("retina", "retina_config_1.py")
@nrp.MapRobotSubscriber("camera", Topic('/icub_model/left_eye_camera/image_raw', sensor_msgs.msg.Image))
@nrp.MapSpikeSource("neurons", nrp.map_neurons(range(0, 320), lambda i: nrp.brain.sensors[i]), nrp.dc_source)
@nrp.Robot2Neuron()
# Example TF: get image and fire at constant rate.
# You could do something with the image here and fire accordingly.
def grab_image_retina_1(t, retina, neurons, camera):
    retina = retina.value
    tf = hbp_nrp_cle.tf_framework.tf_lib

    image = camera.value
    if image is not None:
        retina.update(tf.bridge.imgmsg_to_cv2(image))
        magic_row = 70
        for i in xrange(320):
            neurons[i].amplitude = retina.getValue(magic_row, i, 'Output')
