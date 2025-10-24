import os
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import kagglehub

# Download latest version
path = kagglehub.dataset_download("vishweshsalodkar/wild-animals")
img_path = os.path.join(path, 'Animals/Tiger/pexels-photo-3275319.jpg')
print("Path to dataset files:", path)

def read_image():
    img = cv.imread(img_path)
    cv.imshow('image frame', img)
    cv.waitKey(0)

def write_image():
    out_path = os.path.join(os.getcwd(), 'data/output_img.jpg')
    cv.imwrite(out_path, cv.imread(img_path))
    print(f'pass @ {out_path}')

if __name__ == '__main__':
    read_image()
    write_image()