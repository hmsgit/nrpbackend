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
