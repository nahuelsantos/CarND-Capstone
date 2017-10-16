from styx_msgs.msg import TrafficLight
from keras.preprocessing.image import load_img, img_to_array
import cv2
import numpy as np
import keras.backend as K
import rospy
import tensorflow as tf
from graph_utils import load_graph

IMAGE_SIZE = 224

ready_for_classification = False

class TLClassifier(object):
    def __init__(self, model_name):

        self.ready_for_classification = False

        K.set_image_dim_ordering('tf')

        self.graph, ops = load_graph('light_classification/Models/{}.pb'.format(model_name))
        self.sess = tf.Session(graph = self.graph)

        self.learning_phase_tensor = self.graph.get_tensor_by_name('fire9_dropout/keras_learning_phase:0')
        self.op_tensor = self.graph.get_tensor_by_name('softmax/Softmax:0')
        self.input_tensor = self.graph.get_tensor_by_name('input_1:0')

        self.pred_dict = {0: TrafficLight.UNKNOWN,
                    1: TrafficLight.RED,
                    2: TrafficLight.GREEN}

        # This is used to prevent the classification is done before the classifier initialized  
        self.ready_for_classification = True

    def get_classification(self, image):

        try:
            """Determines the color of the traffic light in the image
            Args:
                image (cv::Mat): image containing the traffic light
            Returns:
                int: ID of traffic light color (specified in styx_msgs/TrafficLight)
            """
            preds = None
            if self.ready_for_classification:
                # Convert image to RGB to be compatible with the model.
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                # The image should arrive already with size 224x224. Resize it to be sure
                image = cv2.resize(image, (IMAGE_SIZE, IMAGE_SIZE))
                # Adjust image values converting the type and normalizing it.
                image = image.astype(K.floatx())
                image /= 255.0
                
                image = np.expand_dims(image, axis=0)

                with self.graph.as_default() as graph:
                    feed_dict = {self.input_tensor: image, self.learning_phase_tensor: False}
                    preds = self.sess.run(self.op_tensor, feed_dict)

                pred = np.argmax(preds)
                # Convert the predicted class in the "format" required by the detector (as defined in the pred_dict)
                pred = self.pred_dict[pred]

            else:
                pred = TrafficLight.UNKNOWN

            return pred
        except Exception as e:
            print("Error during classification :{}".format(e))
