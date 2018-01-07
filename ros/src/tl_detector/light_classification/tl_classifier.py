from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
from utils import label_map_util

CKPT = 'model/output/frozen_inference_graph.pb'
PATH_TO_LABELS = 'data/tf_records/label_map.pbtxt'

NUM_CLASSES = 14

class TLClassifier(object):
    def __init__(self):
        self.detection_graph = tf.Graph()

        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map,
                                                                    max_num_classes=NUM_CLASSES,
                                                                    use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self.conf_prob = 0.5 # detection should greater than this confidence probability

        with self.detection_graph.as_default():
            #Load a (frozen) Tensorflow model into memory.
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

            # Definite input and output Tensors for detection_graph
            self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as sess:
                
                (boxes, scores, classes, num) = sess.run([self.detection_boxes, self.detection_scores,
                                                          self.detection_classes, self.num_detections],
                                                         feed_dict={image_tensor: img})

                boxes = np.squeeze(boxes)
                scores = np.squeeze(scores)
                classes = np.squeeze(classes).astype(np.int32)

                max_box = .0;
                nearest_light_num = -1;
                for i in range(boxes.size()):
                    if scores[i] > self.conf_prob:
                        box = boxes[i]
                        light_state = self.category_index[classes[i]]['name']
                        box_area = (box[2] - box[0]) * (box[3] - box[1])
                        if (box_area > max_box):
                            max_box = box_area
                            nearest_light_num = i;
                            nearest_light_state  = light_state;

                if (nearest_light_num > 0):
                    if light_state == "Red" :
                        return TrafficLight.RED
                    if light_state == "Green" :
                        return TrafficLight.GREEN
                    if light_state == "Yellow" :
                        return TrafficLight.YELLOW
                else
                    return TrafficLight.UNKNOWN
