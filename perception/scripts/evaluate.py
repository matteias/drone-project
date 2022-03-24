#!/usr/bin/env python
from __future__ import print_function

import argparse
from datetime import datetime
import os

import torch
from torch import nn
from torchvision.datasets import CocoDetection
import torchvision.transforms.functional as TF
from PIL import Image
import matplotlib.pyplot as plt

import numpy as np
import cv2 as cv


import utils
from detector import Detector

import time

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
    def __init__(self, device = "cpu", model_path = 'network1.pt'):
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
        b = self.bbs[0][0]

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

        start = np.array((y_start, x_start))
        extracted = im_array[y_start:y_end,x_start:x_end,:]
        return start, extracted


def point_corr(kp1, des1, kp2, des2, img1, img2): #get corresponding points between two sift point lists
    # CALCULATE CORRESPONDENCES AND HOMOGRAPHY
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1,des2,k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)
    #print(good)

    success = False

    MIN_MATCH_COUNT = 5

    if len(good)>MIN_MATCH_COUNT:
        success = True
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts,M)
        img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)
    else:
        print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
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
        return src_pts, dst_pts
    else:
        return None, None


def main():
    net = network()
    sift = cv.SIFT_create()

    image = cv.imread('airport/img_15.jpg')

    net.get_bbs(image)
    start1, extracted1 = net.bb_params()

    img1 = cv.cvtColor(extracted1,cv.COLOR_BGR2GRAY)

    kp1, des1 = sift.detectAndCompute(img1, None)

    ref = cv.imread('airport/img_20.jpg')


    net.get_bbs(ref)
    start2, extracted2 = net.bb_params()

    img2 = cv.cvtColor(extracted2,cv.COLOR_BGR2GRAY)

    tic = time.perf_counter()
    kp2, des2 = sift.detectAndCompute(img2, None)
    src, des = point_corr(kp1, des1, kp2, des2, img1, img2)
    toc = time.perf_counter()

    print(toc-tic)

    #print(src+start1)
    #print(des+start2)





if __name__ == '__main__':
    main()
