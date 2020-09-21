import cv2
import time
import paho.mqtt.client as mqtt
import json

broker_address="172.16.116.133"
cap = cv2.VideoCapture(-1)
r2, img2 = cap.read()
img_list2 = img2.tolist()
var2 = json.dumps(img_list2)
# print(var2)

def on_connect(client, userdata, flags, rc):
  print("Connected with result code "+str(rc))
  client.subscribe("topic/switch")

def on_message(client, userdata, msg):
    r, img = cap.read()
    img_list = img.tolist()
    var = json.dumps(img_list)
    client.publish("topic/send_photo", var)

    # received integer count of boxes
    cou = msg.payload.decode()
    print(cou)

client = mqtt.Client()
client.connect(broker_address,1883,60)
client.on_connect = on_connect
client.on_message = on_message
# print("vakul")
client.publish("topic/send_photo", var2)
client.loop_forever()
