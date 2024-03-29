import cv2
import numpy as np
import time
import imutils
import argparse
import math
from imutils.video import VideoStream
import os

old_cX = None
old_cY = None


'''
Tuning APIs
'''
def trackbar_callback(val):
    """
    Trackbar callback do nothing
    """
    pass

def get_parameters():
    """
    Generate hough parameter values
    """
    _dp = max(cv2.getTrackbarPos('dp (x10)', 'Hough parameter tune') / 10, 1)
    _minDist = max(cv2.getTrackbarPos('minDist', 'Hough parameter tune'), 1)
    _param1 = max(cv2.getTrackbarPos('param1', 'Hough parameter tune'), 1)
    _param2 = max(cv2.getTrackbarPos('param2', 'Hough parameter tune'), 1)
    _minRadius = cv2.getTrackbarPos('minRadius', 'Hough parameter tune')
    _maxRadius = cv2.getTrackbarPos('maxRadius', 'Hough parameter tune')
    
    return _dp, _minDist, _param1, _param2, _minRadius, _maxRadius

def param_trackbar():
    cv2.namedWindow('Hough parameter tune')
    cv2.createTrackbar('dp (x10)', 'Hough parameter tune', 1, 20, trackbar_callback)
    cv2.createTrackbar('minDist', 'Hough parameter tune', 1, 400, trackbar_callback)
    cv2.createTrackbar('param1', 'Hough parameter tune', 1, 400, trackbar_callback)
    cv2.createTrackbar('param2', 'Hough parameter tune', 1, 200, trackbar_callback)
    cv2.createTrackbar('minRadius', 'Hough parameter tune', 0, 400, trackbar_callback)
    cv2.createTrackbar('maxRadius', 'Hough parameter tune', 0, 400, trackbar_callback)


def get_threshold():
    """
    Generate threshold values
    """
    lh = cv2.getTrackbarPos('Lower H', 'Threshold trackbar tune')
    uh = cv2.getTrackbarPos('Upper H', 'Threshold trackbar tune')

    ls = cv2.getTrackbarPos('Lower S', 'Threshold trackbar tune')
    us = cv2.getTrackbarPos('Upper S', 'Threshold trackbar tune')
    
    lv = cv2.getTrackbarPos('Lower V', 'Threshold trackbar tune')
    uv = cv2.getTrackbarPos('Upper V', 'Threshold trackbar tune')
    
    return np.array([lh, ls, lv], dtype='uint8'),\
        np.array([uh, us, uv], dtype='uint8')

def threshold_trackbar():
    cv2.namedWindow('Threshold trackbar tune')
    cv2.createTrackbar('Lower H', 'Threshold trackbar tune', 0, 179, trackbar_callback)
    cv2.createTrackbar('Upper H', 'Threshold trackbar tune', 0, 179, trackbar_callback)
    cv2.createTrackbar('Lower S', 'Threshold trackbar tune', 0, 255, trackbar_callback)
    cv2.createTrackbar('Upper S', 'Threshold trackbar tune', 0, 255, trackbar_callback)
    cv2.createTrackbar('Lower V', 'Threshold trackbar tune', 0, 255, trackbar_callback)
    cv2.createTrackbar('Upper V', 'Threshold trackbar tune', 0, 255, trackbar_callback)

'''
Main detection function
'''
img_num = 0
def dropzone_detect():
    global old_cX, old_cY
    # camera resolution width and height parameters
    width = 200
    height = 200
    
    # parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-va', "--video_address", type=int, default=0, help='Video source address')
    parser.add_argument('-tc', "--tune_color", action='store_true', help='Tune color threshold mode')
    parser.add_argument('-th', "--tune_hough", action='store_true', help='Tune hough parameters mode')
    args = parser.parse_args()

    #img = cv2.imread('6.jpg')

    if args.video_address == -1:
        cam = VideoStream(usePiCamera=False).start()
    else:
        cam = VideoStream(src=args.video_address).start()
    # cam = cv2.VideoCapture("/home/hanssms/cam.mp4")

    time.sleep(2.)

    if args.tune_hough:
        param_trackbar()

    if args.tune_color:
        threshold_trackbar()
    else:
        # set lower and upper hsv threshold in red
        lower = np.array([44, 21, 68], dtype='uint8')
        upper = np.array([179, 255, 154],  dtype='uint8')

    while True:
        # pre process
        img = cv2.imread("data5.jpg")
        #img = cam.read() 
        img = imutils.resize(img, width=height)
        img_disp = img.copy()
        blur = cv2.GaussianBlur(img, (5, 5), 0)

        # color filtering
        if args.tune_color:
            lower, upper = get_threshold() # uncomment to tune HSV range
        
        # hough parameters
        if args.tune_hough:
            hough_dp, hough_minDist, hough_param1, hough_param2, hough_minRadius, hough_maxRadius = get_parameters() 

        frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        frame = cv2.inRange(frame, lower, upper)
        frame = cv2.bitwise_and(blur, blur, mask=frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # global img_num
        # if img_num%5 ==0:
        #     img_data = "dataset_{}.jpg".format(str(img_num))
        #     cv2.imwrite(os.path.join("/home/hanssms/dataset", img_data), img)
        # img_num += 1


        # circle detection using hough transform
        if args.tune_hough:
            circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=hough_dp, minDist=hough_minDist,
                param1=hough_param1, param2=hough_param2,
                minRadius=hough_minRadius, maxRadius=hough_maxRadius)
        else:
            circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=1.5, minDist=131,
                param1=196, param2=32, #23
                minRadius=0, maxRadius=width)

        largest_circle_radius = 0
        largest_circle_center = None

        # display bounding circle
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv2.circle(img_disp, center, 1, (255,255,255), 3)
                cv2.putText(img_disp, "center", (i[0] - 20, i[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # circle outline
                radius = (i[2])
                cv2.circle(img_disp, center, radius, (0,255,0), 3)
                
                if radius > largest_circle_radius:
                    largest_circle_radius = radius
                    largest_circle_center = center
                    
        # transform pixel coordinate system to screen coordinate system
        if largest_circle_center is not None:
            cX = largest_circle_center[0] - width/2
            cY = height/2 - largest_circle_center[1]
        else:
            cX = float('nan')
            cY = float('nan')
        
        cNorthAngle = math.degrees(math.atan2(cX, cY))
        cNormalAngle = math.degrees(math.atan2(cY, cX))

        if ((old_cX != cX) and (old_cY != cY)):
            print("x coordinate", cX)
            print("y coordinate", cY)
            print("north angle", cNorthAngle)
            print("normal angle", cNormalAngle)
            old_cX = cX
            old_cY = cY

        cv2.imshow("Camera", img_disp)
        cv2.imshow("Mask", frame)
    
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    try:
        dropzone_detect()
    except KeyboardInterrupt:
        pass
