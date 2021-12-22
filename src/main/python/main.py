# import cv2
from cv2 import cv2 #comment this and uncomment top if cv2 member errors
import numpy
import time
import math
from enum import Enum

from networktables import NetworkTables

NetworkTables.initialize()
vision_nt = NetworkTables.getTable('Vision')

averagePixel = vision_nt.getEntry("pixel-avg")

def main():

    #starts video capture in opencv lib
    vid = cv2.VideoCapture(0) 

    while True:
        time2, input_img = vid.read()

        if time2 == 0: #if the same frame wait for next
            continue

        input_img.
        averagePixel.setNumber(cv2.meanStdDev(input_img))
        # if len(pipeline.cascade_classifier_output) >= 1:
            # averagePixel.setNumber(pipeline.cascade_classifier_output[0][3]) #gives width of first face to stream, used to determine if it exists (you can expand on this)
        # else:
            # averagePixel.setNumber(0)
