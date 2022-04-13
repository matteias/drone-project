#!/usr/bin/env python3.7
from evaluate import *
import rospy
from sensor_msgs.msg import Image
import PIL
import numpy as np
import cv2 as cv
#from tf.transformations import quaternion_about_axis
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Quaternion, TransformStamped
#import tf2_ros

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


def makePoseWithCovariance(rvec,tvec,cat_id):
    markerMsg = Marker()
    markerMsg.id = cat_id



#def makeMarkerArray(bbs):
#    pass



rospy.init_node('perception')
img_sub = rospy.Subscriber('/cf1/camera/image_raw', Image, image_callback)
bb_pub = rospy.Publisher('/perception/image', Image, queue_size=2)
pose_pubby = rospy.Publisher('/perception/sign_pose', MarkerArray,queue_size=3)



if __name__== "__main__":
    net = network()
    sift = cv.SIFT_create()

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        img = image
        #print(img.shape)
        bbs = net.get_bbs(img)
        markerMsg = Marker()
        markerMsg.header.stamp = t
        start, extracted, detected, cat, cat_id = net.bb_params()

        #print(detected)

        if detected:
            try:
                rvec, tvec = get_pose(extracted, cat, sift, start)
            except:
                rvec, tvec = None, None
            #print(rvec)
            if tvec is not None:
                #print(tvec[2])
                markerMsg.header.frame_id = 'camera_link'
                markerMsg.id = cat_id

                markerMsg.pose.pose.position = Point(*tvec)
                angle = np.linalg.norm(rvec)
                rvec /= angle
                quat = Quaternion()
                quat.x = rvec[0]*np.sin(angle/2)
                quat.y = rvec[1]*np.sin(angle/2)
                quat.z = rvec[2]*np.sin(angle/2)
                quat.w = np.cos(angle/2)
                #print(quat)

                markerMsg.pose.pose.orientation = quat
                #markerMsg.pose.pose.orientation = Quaternion(*rvec)


                markerArray = MarkerArray()
                markerArray.header = markerMsg.header
                markerArray.markers = [markerMsg]

                #print(markerArray)
                pose_pubby.publish(markerArray)
            #print(cat)



        '''
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
            '''
        rate.sleep()

    rospy.spin()
