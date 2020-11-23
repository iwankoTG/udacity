from styx_msgs.msg import TrafficLight
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        #pass
        SSD_GRAPH_FILE = 'ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
        detection_graph = load_graph(SSD_GRAPH_FILE)

        # copied from sec10:CarND-Object-Detection-Lab
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

        confidence_cutoff = 0.2

        sess = tf.Session(graph=detection_graph)



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
                                        feed_dict={image_tensor: image_np})

        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        # Filter boxes with a confidence score less than `confidence_cutoff`
        boxes, scores, classes = filter_boxes2(self.confidence_cutoff, boxes, scores, classes)

        # This converts the coordinates actual location on the image.
        width, height = image.size
        box_coords = to_image_coords(boxes, height, width)

        stete = get_traffic_light_state(image, box_coords)

        return TrafficLight.UNKNOWN

    def load_graph(graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def filter_boxes2(min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score` and class of traffic lights(10)"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score and classes[i] == 10:
                idxs.append(i)

        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(boxes, height, width):
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

    def get_traffic_light_state(image, boxes):
        """Classify traffic lights in red, green and yellow"""
        state_red = 0
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            box_height = int((top - bot)/3.0)
            box_red = np.array(image.crop((left, bot, right, bot+box_height)))
            box_ylw = np.array(image.crop((left, bot+box_height, right, top-box_height)))
            box_grn = np.array(image.crop((left, top-box_height, right, top)))
            #print(bot, left, top, right)
            #print(box_height)
            #print(box_red.shape)
            #print(np.max(box_red[2:-2,2:-2,0]), np.max(box_red[2:-2,2:-2,1]), np.max(box_red[2:-2,2:-2,2]))
            #print(np.mean(box_red[2:-2,2:-2,0]), np.mean(box_red[2:-2,2:-2,1]), np.mean(box_red[2:-2,2:-2,2]))
            if(np.max(box_red[2:-2,2:-2,0]) > 240): state_red +=1

        if state_red > 0:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKOWN
