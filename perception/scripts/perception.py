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
import copy

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


def draw_bb(s, e, imageBB, color,pad):

    x_start = s[0]
    y_start = s[1]

    #width = int(b["width"])
    #height = int(b["height"])
    x_end = e[0]
    y_end = e[1]

    h = image.shape[0]
    w = image.shape[1]
    x_start = max(pad,min(w-pad,x_start))
    x_end = max(pad,min(w-pad,x_end))
    y_start = max(pad,min(h-pad,y_start))
    y_end = max(pad,min(h-pad,y_end))
    #print((h,w))
    bb_img = imageBB

    bb_img[y_start:y_end,x_start-pad:x_start+pad,:] = color
    bb_img[y_start:y_end,x_end-pad:x_end+pad,:] = color
    bb_img[y_start-pad:y_start+pad,x_start:x_end,:] = color
    bb_img[y_end-pad:y_end+pad,x_start:x_end,:] = color

    #print(bb_img[:x_end,y_start,:])
    #bb_img = cv2.rectangle(image, start_point, end_point, color, thickness)
    return bb_img

def draw_allbb(starts, ends, img, color=np.array([0,0,255]), pad = int(2)):

    for i in range(len(starts)):
        img = draw_bb(starts[i], ends[i], img, color,pad)

    return img


def process_bbparams(s, e, cats, cat_ids):
    N = len(starts)

    threshold = 1/2 #threshold for overlapping of bounding boxes (S/U)
    IoUmat = np.zeros((N,N))
    mergeCount = np.ones(N)
    shouldInclude = np.ones(N, dtype = bool)
    for i in range(N):
        for j in range(N):
            if not i == j:
                snittS = np.maximum(s[i], s[j])
                snittE = np.minimum(e[i], e[j])
                snittDelta = snittE-snittS
                if np.min(snittDelta) <= 0:
                    IoU = 0
                else:#hej!
                    I = snittDelta[0]*snittDelta[1]
                    A1 = np.prod(e[i]-s[i])
                    A2 = np.prod(e[j]-s[j])
                    #U = A1 + A2 - 2*I
                    #IoU = I / U
                    IoU = np.min(np.array([A1,A2]))/I
                IoUmat[i,j] = IoU

    merged = True
    while merged:
        merged = False
        for i in range(N):
            if shouldInclude[i]:
                for j in range(N):
                    if IoUmat[i,j] > threshold and cat_ids[i] == cat_ids[j] and shouldInclude[j]:
                        s[i] = np.minimum(s[i], s[j])
                        e[i] = np.maximum(e[i], e[j])
                        shouldInclude[j] = False
                        merged = True

    s_new = []
    e_new = []
    cats_new = []
    cat_ids_new = []
    for i, val in enumerate(shouldInclude):
        if val:
            s_new.append(s[i])
            e_new.append(e[i])
            cats_new.append(cats[i])
            cat_ids_new.append(cat_ids[i])

    print("removed BBs: " +  str(len(cat_ids)-len(cat_ids_new)) + '\n  new cats: ' + str(cat_ids_new))
    return s_new, e_new, cats_new, cat_ids_new

def extract_signs(starts, ends, im_array): #extracts images of only signs
    extracteds = []
    for i in range(len(starts)):
        extracted = im_array[starts[i][1]:ends[i][1],starts[i][0]:ends[i][0],:]
        extracteds.append(extracted)
    return extracteds



rospy.init_node('perception')
img_sub = rospy.Subscriber('/cf1/camera/image_raw', Image, image_callback)
bb_pub = rospy.Publisher('/perception/image', Image, queue_size=2)
pose_pubby = rospy.Publisher('/perception/sign_pose', MarkerArray,queue_size=3)

#newbb_pub = rospy.Publisher('/perception/image2', Image, queue_size=2)



if __name__== "__main__":
    net = network()
    sift = cv.SIFT_create()

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        t = rospy.Time.now()
        img = copy.deepcopy(image)

        bb_img = copy.deepcopy(img)
        #print(img.shape)
        net.get_bbs(img)
        markerMsg = Marker()
        markerMsg.header.stamp = t

        starts, ends, cats, cat_ids, detected = net.bb_params()
        s_new, e_new, cats_new, ids_new = process_bbparams(starts, ends, cats, cat_ids)

        bb_img = draw_allbb(s_new, e_new, bb_img, color=np.array([255, 0, 0]) , pad = int(4))
        bb_img = draw_allbb(starts, ends, bb_img, color=np.array([0, 0, 255]) , pad = int(1))

        starts, ends, cats, cat_ids = s_new, e_new, cats_new, ids_new
        sign_imgs = extract_signs(starts, ends, img)

        if detected:

            markerArray = MarkerArray()
            markerArray.header = markerMsg.header
            marker_list = []

            for i, extracted in enumerate(sign_imgs):
                try:
                    rvec, tvec = get_pose(extracted, cats[i], sift, starts[i])
                    print(tvec)
                except:
                    rvec, tvec = None, None

                if tvec is not None:
                    #print(tvec[2])
                    markerMsg.header.frame_id = 'camera_link'
                    markerMsg.id = cat_ids[i]

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
                    marker_list.append(markerMsg)


            markerArray.markers = marker_list
            print(cat_ids)

                    #print(markerArray)


            pose_pubby.publish(markerArray)
        contig = np.ascontiguousarray(bb_img)
        image_raw.data = contig.tostring()
        bb_pub.publish(image_raw)
                #print(rvec)
                #print(tvec)

            #print(rvec)

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
