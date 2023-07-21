# import cv2 to capture videofeed
import cv2

import numpy as np

# attach camera indexed as 0
camera = cv2.VideoCapture(0)

# setting framewidth and frameheight as 640 X 480
camera.set(3 , 640)
camera.set(4 , 480)

# loading the mountain image
mountain = cv2.imread('mount everest.jpg')

# resizing the mountain image as 640 X 480
fourcc = cv2.VideoWriter_fourcc(*'XVID')
mountain = cv2.VideoWriter('output.avi', fourcc, 20.0, (640,480))

while True:

    # read a frame from the attached camera
    status , frame = camera.read()

    # if we got the frame successfully
    if status:

        # flip it
        frame = cv2.flip(frame , 1)

        # converting the image to RGB for easy processing
        frame_rgb = cv2.cvtColor(frame , cv2.COLOR_BGR2RGB)

        # creating thresholds
        lower_bound = np.array([])
        upper_bound = np.array([])

        # thresholding image
        lower_bound = lower_bound + upper_bound
        cv2.imshow("lower_bound", lower_bound)
        
        # bitwise and operation to extract foreground / person
        lower_bound = cv2.morphologyEx(lower_bound, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        lower_bound = cv2.morphologyEx(lower_bound, cv2.MORPH_DILATE, np.ones((3,3), np.uint8))
        # final image
        upper_bound = cv2.bitwise_not(upper_bound)
        # show it
        cv2.imshow('frame' , frame)

        # wait of 1ms before displaying another frame
        code = cv2.waitKey(1)
        if code  ==  32:
            break

# release the camera and close all opened windows
camera.release()
cv2.destroyAllWindows()
