#!/usr/bin/env python
from evaluate import get_bbs
import rospy
from sensor_msgs.msg import Image
import PIL
import numpy as np
import cv2

image = None
def image_callback(msg):
    global image, image_raw, imageBB
    #img = ros_numpy.numpify(msg)
    #image = PIL.Image.fromarray(img)
    #dtype_class, channels = name_to_dtypes[msg.encoding]
    #RBG8 channel
    channels = 3
    dtype = np.uint8

    shape = (msg.height, msg.width, channels)
    arr = np.fromstring(msg.data, dtype=dtype).reshape(shape)

    #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    #encoding = (dtype, channels)

    #im = Image(encoding=encoding)
    #im.width = msg.width
    #im.height = msg.height

    '''
    contig = np.ascontiguousarray(arr)
    im.data = contig.tostring()
    im.step = contig.strides[0]
    im.is_bigendian = (
        arr.dtype.byteorder == '>' or
        arr.dtype.byteorder == '=' and sys.byteorder == 'big')
    '''
    #print(im)

    image = arr
    image_raw = msg


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



rospy.init_node('bb_publisher')
img_sub = rospy.Subscriber('/cf1/camera/image_raw', Image, image_callback)
bb_pub = rospy.Publisher('/perception/image', Image, queue_size=2)

if __name__== "__main__":

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if image is not None:
            bbs = get_bbs(image)
            imageBB = image
            for bb in bbs:
                if bb:
                    for b in bbs:
                        for bstar in b:
                            #print (b)
                            #for bstar in b:
                            #print(bstar["category"])
                            imageBB = draw_bb(bstar,imageBB)
                            contig = np.ascontiguousarray(imageBB)
                            image_raw.data = contig.tostring()
            bb_pub.publish(image_raw)
        rate.sleep()

    rospy.spin()
