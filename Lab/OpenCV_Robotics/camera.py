import os
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

# open camera
def camera_init():
    cap = cv.VideoCapture(0)

    # check for successful camera connection
    if not cap.isOpened():
        print('Error connecting to default camera')
        return
    
    # set camera properties
    cap.set(cv.CAP_PRO)

if __name__ == '__main__':
    camera_init()