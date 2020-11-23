from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #pass
        SSD_GRAPH_FILE = '/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
        detection_graph = self.load_graph(SSD_GRAPH_FILE)

        # copied from sec10:CarND-Object-Detection-Lab
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

        self.confidence_cutoff = 0.3

        self.sess = tf.Session(graph=detection_graph)



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        (boxes, scores, classes) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes],
                                        feed_dict={self.image_tensor: image_np})

        #print(boxes)
        #print(scores)
        #print(classes)
        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores, classes = self.filter_boxes2(self.confidence_cutoff, boxes, scores, classes)

        # This converts the coordinates actual location on the image.
        width = image.shape[1]
        height = image.shape[0]
        box_coords = self.to_image_coords(boxes, height, width)

        state = self.get_traffic_light_state(image, box_coords)

        #return TrafficLight.UNKNOWN
        return state

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def filter_boxes2(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score` and class of traffic lights(10)"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score and classes[i] == 10 and boxes[i][2] - boxes[i][0] > 0.07:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        return box_coords

    def get_traffic_light_state(self, image, boxes):
        """Classify traffic lights in red, green and yellow"""
        state_red = 0
        #print("# of detected boxes:", len(boxes))
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            buf = (top - bot)*0.1
            bot = int(bot + buf)
            top = int(top - buf)
            left = int(left + buf/2)
            right = int(right - buf/2)
            box_height = int((top - bot)/3.0)
            box_red = image[int(bot):int(bot+box_height),int(left):int(right)]
            #box_ylw = image[int(bot+box_height):int(top-box_height),int(left):int(right)] 
            #box_grn = image[int(top-box_height):int(top),int(left):int(right)]
            
            #cv2.imwrite("/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/box_red.jpg", box_red)
            #cv2.imwrite("/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/box_ylw.jpg", box_ylw)
            #cv2.imwrite("/home/workspace/CarND-Capstone/ros/src/tl_detector/light_classification/box_grn.jpg", box_grn)
            #break
            #print(bot, left, top, right)
            #print(box_height)
            #print(box_red.shape)
                        
            #print(np.max(box_ylw[2:-2,2:-2,0]), np.max(box_ylw[2:-2,2:-2,1]), np.max(box_ylw[2:-2,2:-2,2]))
            #print(np.max(box_grn[2:-2,2:-2,0]), np.max(box_grn[2:-2,2:-2,1]), np.max(box_grn[2:-2,2:-2,2]))
            #print(np.mean(box_red[:,:,0]), np.mean(box_red[:,:,1]), np.mean(box_red[:,:,2]))
            
            val = np.mean(box_red[:,:,2])/(np.mean(box_red[:,:,0]) + np.mean(box_red[:,:,1]) + np.mean(box_red[:,:,2]))
            #print(val)
            #if(np.max(box_red[:,:,2]) > 245): state_red +=1
            if(val > 0.5): state_red +=1

        if state_red > 0:
            print("********BREAK!!!*********")
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN
