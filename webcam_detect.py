# Code adapted from Tensorflow Object Detection Framework
# https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb
# Tensorflow Object Detection Detector

import numpy as np
import tensorflow as tf
import cv2
import time
import json
import paho.mqtt.client as mqtt
broker_address="172.16.116.133"


class DetectorAPI:
    def __init__(self, path_to_ckpt):
        self.path_to_ckpt = path_to_ckpt

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.default_graph = self.detection_graph.as_default()
        self.sess = tf.Session(graph=self.detection_graph)

        # Definite input and output Tensors for detection_graph
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def processFrame(self, image):
        # Expand dimensions since the trained_model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image, axis=0)
        # Actual detection.
        start_time = time.time()
        (boxes, scores, classes, num) = self.sess.run(
            [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
            feed_dict={self.image_tensor: image_np_expanded})
        end_time = time.time()

        print("Elapsed Time:", end_time-start_time)

        im_height, im_width,_ = image.shape
        boxes_list = [None for i in range(boxes.shape[1])]
        for i in range(boxes.shape[1]):
            boxes_list[i] = (int(boxes[0,i,0] * im_height),
                        int(boxes[0,i,1]*im_width),
                        int(boxes[0,i,2] * im_height),
                        int(boxes[0,i,3]*im_width))

        return boxes_list, scores[0].tolist(), [int(x) for x in classes[0].tolist()], int(num[0])

    def close(self):
        self.sess.close()
        self.default_graph.close()


if __name__ == "__main__":

    # PREPROCESSING
    model_path = 'faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb'
    #model_path = 'ssd_mobilenet_v1_coco_2017_11_17/frozen_inference_graph.pb'

    odapi = DetectorAPI(path_to_ckpt=model_path)
    threshold = 0.7
    #CLIENT MQTT
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        client.subscribe("topic/send_photo")

    def on_message(client, userdata, msg):
        # r, img = cap.read()
        # print("chirag")
        # print(type(odapi))
        str = msg.payload.decode()
        # print(str)
        str_list = json.loads(str)
        # print(str_list)
        # print("chirag")
        img = np.array(str_list)
        # print(type(img))
        #img = cv2.resize(img, (1280, 720))
        # img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # print(type(img))
        boxes, scores, classes, num = odapi.processFrame(img)

        # Visualization of the results of a detection.
        cou=0
        # print("chirag")

        for i in range(len(boxes)):
            # Class 1 represents human
            if classes[i] == 1 and scores[i] > threshold:
            	cou=cou+1
                #box = boxes[i]
                #cv2.rectangle(img,(box[1],box[0]),(box[3],box[2]),(255,0,0),2)

        #cv2.imshow("preview", img)
        # print(cou)
        #key = cv2.waitKey(20)
        # cou *= 10
        # cou = ""+cou
        # cou += "#"
        print(cou)
        client.publish("topic/cam_result", cou)

    client = mqtt.Client()
    client.connect(broker_address,1883,60)
    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_forever()
