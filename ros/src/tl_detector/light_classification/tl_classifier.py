from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import os
import rospy


MODEL_DIR = 'light_classification/models/'
IMG_DIR = 'light_classification/img/'
TRAFFIC_LIGHT_CLASSES = 10


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # object detection: faster_rcnn_inception_v2
        # from Tensorflow detection model zoo:                              
        # https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md
        self.detector = MODEL_DIR + 'faster_rcnn_inception_v2.pb'
        self.sess= self.load_graph(self.detector)
        detection_graph = self.sess.graph
        
        # The input placeholder for the image.
        # 'get_tensor_by_name' returns the Tensor with the associated name in the Graph.
        self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')

        # the first decoding
        test_image = cv2.imread(IMG_DIR + 'image3.png')
        image_np, box_coords, classes, scores = self.detect_tl(test_image)
        # Traditional traffic light classifier
        pred_image, is_red = self.classify_red_tl(image_np, box_coords, classes, scores)
        rospy.loginfo("DEBUG: stage 4")
        if is_red:
            rospy.loginfo("Classifier: RED")
        else:
            rospy.loginfo("Classifier: NOT RED")                  
        cv2.imwrite(IMG_DIR + 'pred_image.png', pred_image)
        rospy.loginfo("TensorFlow Initiation: Done")
        self.num_image = 1
        
        
    def load_graph(self, graph_file):
        config = tf.ConfigProto(log_device_placement=False)
        config.gpu_options.allow_growth = True
        session = tf.Session(config=config)
        with tf.Session(graph=tf.Graph(), config=config) as sess:
            gd = tf.GraphDef()
            with tf.gfile.Open(graph_file, 'rb') as f:
                data = f.read()
                gd.ParseFromString(data)
            tf.import_graph_def(gd, name='')
#             ops = sess.graph.get_operations()
            return sess
      
 # Detect traffic light box
    def detect_tl(self, image):
        trt_image = np.copy(image)
        image_np = np.expand_dims(np.asarray(trt_image, dtype=np.uint8), 0)
        rospy.loginfo("DEBUG: stage 0")
        # run detection.
        (boxes, scores, classes) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                            feed_dict={self.image_tensor: image_np})
        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
    
        threshold = 0.8
        # Filter traffic light boxes with threshold
        rospy.loginfo("DEBUG: stage 1")
        boxes, scores, classes = self.filter_boxes(threshold, boxes, scores, 
                                                   classes, keep_classes=[TRAFFIC_LIGHT_CLASSES])
        # Convert the normalized box coordinates(0~1) to image coordinates
        image_np = np.squeeze(image_np)
        width = image_np.shape[1]
        height = image_np.shape[0]
        rospy.loginfo("DEBUG: stage 2")
        box_coords = self.to_image_coords(boxes, height, width)
        rospy.loginfo("DEBUG: stage 3")
        
# Filter the boxes which detection confidence lower than the threshold        
    def filter_boxes(self, min_score, boxes, scores, classes, keep_classes):
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                if ((keep_classes is None) or (int(classes[i]) in keep_classes)):
                    idxs.append(i)
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes
       
# Convert the normalized box coordinates (0~1) to image coordinates
    def to_image_coords(self, boxes, height, width):
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
        return box_coords
        
#Draw bounding box on traffic light, and detect if it is RED
    def classify_red_tl(self, image_np, boxes, classes, scores, thickness=5):
        for i in range(len(boxes)):
            rospy.loginfo("DEBUG: stage 3.1")
            bot, left, top, right = boxes[i, ...]
            class_id = int(classes[i])
            score = scores[i]
            h = top - bot
            w = right - left
            if h <= 1.5 * w:
                continue # Truncated Traffic Ligth box
            cv2.rectangle(image_np,(left, top), (right, bot), (255, 43, 255), thickness) # BGR format for color
            tl_img = image_np[int(bot):int(top), int(left):int(right)]

            tl_img_simu = self.select_red_simu(tl_img) # SELECT RED
            tl_img_real = self.select_lighton_real(tl_img) # SELECT TL
            tl_img = (tl_img_simu + tl_img_real) / 2

            gray_tl_img = cv2.cvtColor(tl_img, cv2.COLOR_RGB2GRAY)
            nrows, ncols = gray_tl_img.shape[0], gray_tl_img.shape[1]
            # compute center of mass of RED points
            mean_row = 0
            mean_col = 0
            npoints = 0
            for row in range(nrows):
                for col in range(ncols):
                    if (gray_tl_img[row, col] > 0): 
                        mean_row += row
                        mean_col += col
                        npoints += 1
            if npoints > 0:
              mean_row = float(mean_row / npoints) / nrows
              mean_col = float(mean_col / npoints) / ncols

              # Get the normalized center of mass of RED points 
              # Use the location of light to detect the color, RED is in the upper part of the box
              if npoints > 10 and mean_row < 0.33:
                  rospy.loginfo("RED Light Detection Confidance: %.2f", score)  
                  return image_np, True
        return image_np, False        
        

# select RED mask in simulation situation
    def select_red_simu(self, img): # BGR
        lower = np.array([ 0,   0, 200], dtype="uint8")
        upper = np.array([ 55, 55, 255], dtype="uint8")
        red_mask = cv2.inRange(img, lower, upper)
        return cv2.bitwise_and(img, img, mask = red_mask)

# select Traffic Lighton area(HLS: high L and high S) in real situation
# for camera without polarization filter
    def select_lighton_real(self, img): # HLS for real
        hls_img = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        lower = np.array([ 50,   150, 150], dtype="uint8")
        upper = np.array([ 100, 255, 255], dtype="uint8")
        tl_mask = cv2.inRange(hls_img, lower, upper)
        return cv2.bitwise_and(img, img, mask = tl_mask)
    
        
        
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #implement light color prediction
        image_np, box_coords, classes, scores = self.detect_tl(image)        
        # light color detection
        pred_image, is_red = self.classify_red_tl(image_np, box_coords, classes, scores)
        fimage = DEBUG_DIR + 'image' + str(self.num_image) + '.png'
        #output the predicted image
        cv2.imwrite(fimage, pred_image)
        self.num_image += 1
        #return 'if it is a RED'
        if is_red:
            return TrafficLight.RED
        else:
            return TrafficLight.UNKNOWN

        
