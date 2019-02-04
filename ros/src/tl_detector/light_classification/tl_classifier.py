import tensorflow as tf
import keras
import cv2
import numpy as np
import os
from keras import backend as K
from keras.applications.mobilenet import MobileNet, preprocess_input
from keras import optimizers
from keras.models import Sequential, Model
from keras.layers import Dropout, Flatten, Dense, Input, Activation
from keras.models import model_from_json
from keras.utils.generic_utils import CustomObjectScope
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        path = os.path.realpath(os.path.dirname(__file__))
        model_definition_path = os.path.join(path, 'final.json')
        model_weights_path = os.path.join(path, 'final.h5')

        self.graph = tf.Graph()

        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.3)
        self.session = tf.Session(graph=self.graph, config=tf.ConfigProto(
                                    allow_soft_placement=True, 
                                    log_device_placement=False))

        K.set_session(self.session)

        with self.graph.as_default():
            with open(model_definition_path, 'r') as json_file:
                loaded_model_json = json_file.read()

            with CustomObjectScope({
                    'relu6': keras.applications.mobilenet.relu6, 
                    'DepthwiseConv2D': keras.applications.mobilenet.DepthwiseConv2D}):
                self.model = model_from_json(loaded_model_json)
                self.model.load_weights(model_weights_path)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        # TODO: Train directly on BGR image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (128, 128)) / 255.
        batch = np.expand_dims(image, axis=0)

        with self.graph.as_default():
            prediction = self.model.predict(batch)
            predicted_class = np.argmax(prediction)

        if predicted_class == 0:
            return TrafficLight.UNKNOWN
        elif predicted_class == 1:
            return TrafficLight.RED
        elif predicted_class == 2:
            return TrafficLight.YELLOW
        else:
            assert predicted_class == 3
            return TrafficLight.GREEN
