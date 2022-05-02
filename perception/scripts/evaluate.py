#!/usr/bin/env python3.7
from __future__ import print_function

import argparse
from datetime import datetime
import os

import torch
from torch import nn
#from torchvision.datasets import CocoDetection
#import torchvision.transforms.functional as TF
#from PIL import Image
import matplotlib.pyplot as plt

import numpy as np
import cv2 as cv


import utils
from detector import Detector

#from perception import process_bbparams, extract_signs


def load_dict():
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
    return category_dict


class network():
    def __init__(self, device = "cpu", model_path = 'net_5000.pt'):
        self.device = device

        detector = Detector().to(device)
        map_location = torch.device(device)

        self.detector = utils.load_model(detector, model_path, map_location)
        self.category_dict = load_dict()

    def get_bbs(self,image):
        torch_image, _ = self.detector.input_transform(image, [])

        self.image = image

        images = []
        images.append(torch_image)


        images = torch.stack(images)
        self.images = images.to(self.device)

        self.detector.eval()

        with torch.no_grad():
            self.out = self.detector(self.images).cpu()
            self.bbs = self.detector.decode_output(self.out, 0.5)


    def show_bbs(self):
        for i, test_image in enumerate(self.images):
            figure, ax = plt.subplots(1)
            plt.imshow(self.image)
            plt.imshow(
                self.out[i, 4, :, :],
                interpolation="nearest",
                extent=(0, 640, 480, 0),
                alpha=0.7,
            )

            # add bounding boxes
            utils.add_bounding_boxes(ax, self.bbs[i], self.category_dict)

            plt.show()

    def bb_params(self):
        detected = False
        #print(self.bbs)
        #extracteds = []
        starts = []
        cats = []
        cat_ids = []
        ends = []


        if len(self.bbs) > 0 and len(self.bbs[0]) > 0:
            detected = True
            num_bbs = len(self.bbs[0])
            #print(num_bbs)
            for b in self.bbs[0]:
                im_array = np.array(self.image)
                x_start = int(b["x"])
                y_start = int(b["y"])

                width = int(b["width"])
                height = int(b["height"])
                pad = int(2)
                x_end = x_start + width
                y_end = y_start + height

                h = im_array.shape[0]
                w = im_array.shape[1]
                x_start = max(pad,min(w-pad,x_start))
                x_end = max(pad,min(w-pad,x_end))
                y_start = max(pad,min(h-pad,y_start))
                y_end = max(pad,min(h-pad,y_end))

                start = np.array((x_start, y_start))
                starts.append(start)
                end = np.array((x_end, y_end))
                ends.append(end)
                #size = np.array((height, width)) # width and height wrong?

                #extracted = im_array[y_start:y_end,x_start:x_end,:]
                #extracteds.append(extracted)
                cat = self.category_dict[b['category']]['name']

                cats.append(cat)
                cat_id = b['category']
                #print(cat_id)
                cat_ids.append(cat_id)
            #print(extracteds)

            ##b['category']
        return starts, ends, cats, cat_ids, detected
        #return None, None, None, None, detected


def extract_signs(starts, ends, im_array): #extracts images of only signs
    extracteds = []
    for i in range(len(starts)):
        extracted = im_array[starts[i][1]:ends[i][1],starts[i][0]:ends[i][0],:]
        extracteds.append(extracted)
    return extracteds


def point_corr(kp1, des1, kp2, des2, img1, img2): #get corresponding points between two sift point lists
    # CALCULATE CORRESPONDENCES AND HOMOGRAPHY
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    #print(len(matches))
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    #print(len(good))

    success = False

    MIN_MATCH_COUNT = 5

    if len(good)>MIN_MATCH_COUNT:
        success = True

        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        #print(src_pts.shape)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        try:
            dst = cv.perspectiveTransform(pts,M)
        except:
            return None, None
        #img2 = cv.polylines(img2,[np.int32(dst)],True,255,1, cv.LINE_AA)
    else:
        #print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
        matchesMask = None

    show_plot = False

    if show_plot:
        draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                       singlePointColor = None,
                       matchesMask = matchesMask, # draw only inliers
                       flags = 2)
        img3 = cv.drawMatches(img1,kp1,img2,kp2,good,None,**draw_params)
        plt.imshow(img3, 'gray'),plt.show()

    if success:
        return pts, dst
    else:
        return None, None


def get_pose(extracted, cat, sift, start): # extracted: from camera bb, category: from bounding box
    #print(cat)
    ref_path = 'ref_signs/p' + cat + '.jpg'
    ref_path = ref_path.replace(' ', '_')

    ref = cv.imread(ref_path, cv.IMREAD_GRAYSCALE) # Use category to get reference image
    if cat == 'no stopping and parking' or cat == 'no parking':
        ref = cv.equalizeHist(ref)
        #print('EQUALIZED HIST')


    ref = cv.GaussianBlur(ref,(5,5),cv.BORDER_DEFAULT)
    kp1, des1 = sift.detectAndCompute(ref, None) # reference keypoints
    #print('num ref keypoints: ' + str(len(kp1)))

    try:
        img = cv.cvtColor(extracted,cv.COLOR_BGR2GRAY)
        if cat == 'no stopping and parking' or cat == 'no parking':
            img = cv.equalizeHist(img)
            #print('EQUALIZED HIST')
        #print(cat)
    except:
        return None, None
    kp2, des2 = sift.detectAndCompute(img, None)
    #print('num img keypoints: ' + str(len(kp2)))

    src, des = point_corr(kp1, des1, kp2, des2, ref, img) # image keypoints
    try:
        N = src.shape[0]
    except:
        #print('ERROR: homography not found')
        return None, None

    des = np.reshape(des,(N,2)) # image points
    src = np.reshape(src,(N,2)) # object points
    #print(src)
    src = np.flip(src,axis=1)
    #print(src)
    #print(des)

    #print(src.shape)
    #print(des.shape)

    # turn source image points into 3D pose
    #print(ref.shape)
    hp, wp = ref.shape # pixel height and width

    x0 = hp/2
    y0 = wp/2

    #p_size = 0.123/hp # pixel size in meters [airport]
    p_size = 0.125/wp

    #print()

    src = src-np.array([x0,y0]) # move coordinate system into middle of sign
    src = src*p_size # turn image coordinates into meters
    src = np.concatenate((src, np.zeros((N,1))),axis=1) # add 0 in Z direction


    # bring back original image coordinates from bounding box points
    des = des + start

    mtx = np.load('mtx.npy')
    dist = np.load('dist.npy')

    # Solve Perspective-n-Point for corners
    succ, rvec, tvec = cv.solvePnP(src, des, mtx, dist)


    return rvec, tvec

def main():
    net = network()
    sift = cv.SIFT_create()

    #ref = preprocess_airport()

    #kp1, des1 = sift.detectAndCompute(ref, None) # reference keypoints

    image = cv.imread('test/img_135.jpg')
    #print(image.shape)

    net.get_bbs(image)
    #net.show_bbs()
    starts, ends, cats, cat_ids, detected = net.bb_params()
    #print(starts)

    starts, ends, cats, ids = process_bbparams(starts, ends, cats, cat_ids)

    extracteds = extract_signs(starts, ends, image)

    for i, extracted in enumerate(extracteds):
        rvec, tvec = get_pose(extracted, cats[i], sift, starts[i])
        #print(rvec)
        #print(tvec)

    # FOR PYTHON PLOTTING
    #image = cv.polylines(image,[np.int32(des)],True,(0,255,0),1, cv.LINE_AA)

    #mtx = np.load('mtx.npy')
    #dist = np.load('dist.npy')

    #cv.aruco.drawAxis(image, mtx, dist, rvec, tvec, 0.2)

    #cv.imshow('overlayed', image)

    #if cv.waitKey(0) & 0xff == 27:
    #    cv.destroyAllWindows()
    #extracted = cv.GaussianBlur(extracted,(3,3),cv.BORDER_DEFAULT)


def process_bbparams(s, e, cats, cat_ids):
    N = len(s)

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

    #print("removed BBs: " +  str(len(cat_ids)-len(cat_ids_new)) + '\n  new cats: ' + str(cat_ids_new))
    return s_new, e_new, cats_new, cat_ids_new


if __name__ == '__main__':
    main()
