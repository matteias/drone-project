#!/usr/bin/env python
from evaluate import get_bbs
import rospy
from sensor_msgs.msg import Image
import PIL
import numpy as np
import cv2

file_path = 'test_images/img_3.jpg'

image = Image.open(file_path)

category_dict = [
        {
            "id": 0,
            "name": "no bicycle",
            "supercategory": "traffic sign"
        },
        {
            "id": 1,
            "name": "airport",
            "supercategory": "traffic sign"
        },
        {
            "id": 2,
            "name": "dangerous left",
            "supercategory": "traffic sign"
        },
        {
            "id": 3,
            "name": "dangerous right",
            "supercategory": "traffic sign"
        },
        {
            "id": 4,
            "name": "follow left",
            "supercategory": "traffic sign"
        },
        {
            "id": 5,
            "name": "follow right",
            "supercategory": "traffic sign"
        },
        {
            "id": 6,
            "name": "junction",
            "supercategory": "traffic sign"
        },
        {
            "id": 7,
            "name": "no heavy truck",
            "supercategory": "traffic sign"
        },
        {
            "id": 8,
            "name": "no parking",
            "supercategory": "traffic sign"
        },
        {
            "id": 9,
            "name": "no stopping and parking",
            "supercategory": "traffic sign"
        },
        {
            "id": 10,
            "name": "residential",
            "supercategory": "traffic sign"
        },
        {
            "id": 11,
            "name": "narrows from left",
            "supercategory": "traffic sign"
        },
        {
            "id": 12,
            "name": "narrows from right",
            "supercategory": "traffic sign"
        },
        {
            "id": 13,
            "name": "roundabout",
            "supercategory": "traffic sign"
        },
        {
            "id": 14,
            "name": "stop",
            "supercategory": "traffic sign"
        }
    ]


def draw_bb(b,imageBB):

    x_start = int(b["x"])
    y_start = int(b["y"])

    width = int(b["width"])
    height = int(b["height"])
    pad = int(2)
    x_end = x_start + width
    y_end = y_start + height

    h = image.shape[0]
    w = image.shape[1]
    x_start = max(pad,min(w-pad,x_start))
    x_end = max(pad,min(w-pad,x_end))
    y_start = max(pad,min(h-pad,y_start))
    y_end = max(pad,min(h-pad,y_end))
    print((h,w))
    bb_img = imageBB

    bb_img[y_start:y_end,x_start-pad:x_start+pad,:] = np.array([0,0,255])
    bb_img[y_start:y_end,x_end-pad:x_end+pad,:] = np.array([0,0,255])
    bb_img[y_start-pad:y_start+pad,x_start:x_end,:] = np.array([0,0,255])
    bb_img[y_end-pad:y_end+pad,x_start:x_end,:] = np.array([0,0,255])

    #print(bb_img[:x_end,y_start,:])
    #bb_img = cv2.rectangle(image, start_point, end_point, color, thickness)
    return bb_img



#rospy.init_node('bb_publisher')
#img_sub = rospy.Subscriber('/cf1/camera/image_raw', Image, image_callback)
#bb_pub = rospy.Publisher('/perception/image', Image, queue_size=2)

if __name__== "__main__":

    if image is not None:
        bbs = get_bbs(image)
        imageBB = image
        cats = []
        for bb in bbs:
            if bb:
                for b in bbs:
                    for bstar in b:
                        cats.append(category_dict[bstar["category"]]['name'])
                        #print (b)
                        #for bstar in b:
                        #print(bstar["category"])
                        imageBB = draw_bb(bstar,imageBB)
                        contig = np.ascontiguousarray(imageBB)
                        image_raw.data = contig.tostring()
            print(cats)
            bb_pub.publish(image_raw)
