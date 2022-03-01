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


import utils
from detector import Detector

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

def get_bbs(test_image):
    device="cpu"

    detector = Detector().to('cpu')

    model_path = 'network1.pt'

    map_location=torch.device('cpu')


    detector = utils.load_model(detector, model_path, map_location)

    file_path = 'test_images/img_3.jpg'

    #test_image = Image.open(file_path)

    torch_image, _ = detector.input_transform(test_image, [])


    test_images = []
    test_images.append(torch_image)


    test_images = torch.stack(test_images)
    test_images = test_images.to(device)


    detector.eval()

    with torch.no_grad():
        out = detector(test_images).cpu()
        bbs = detector.decode_output(out, 0.5)
        '''
        for i, test_image in enumerate(test_images):
            figure, ax = plt.subplots(1)
            plt.imshow(test_image.cpu().permute(1, 2, 0))
            plt.imshow(
                out[i, 4, :, :],
                interpolation="nearest",
                extent=(0, 640, 480, 0),
                alpha=0.7,
            )

            # add bounding boxes
            utils.add_bounding_boxes(ax, bbs[i], category_dict)

            plt.show()
        '''
    return bbs
