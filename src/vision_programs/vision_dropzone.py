import rospy
from rasendriya.srv import Dropzone 
import cv2
import numpy as np
import imutils
import argparse
from imutils.video import VideoStream
from std_srvs.srv import SetBool, SetBoolResponse

vision_flag = False

# Sending Dropzone Service
def dropzone_service_client(x,y):
    dropzone_service = rospy.ServiceProxy('/rasendriya/dropzone', Dropzone)
    resp = dropzone_service(x,y)
    return resp.status

def vision_flag_req(req):
    global vision_flag
    vision_flag = req.data
    return SetBoolResponse(True)

def dropzone_detect():
    # initialize ros node
    rospy.init_node('vision_dropzone')
    rospy.wait_for_service('/rasendriya/dropzone')

    # initialize ros publisher
    rate = rospy.Rate(25)

    hit_count = 0

    # camera resolution width and height parameters
    _width = 200
    height = _width

    # parsing arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("integers", metavar='N', type=int, help='Video source address')
    args = parser.parse_args(rospy.myargv()[1:])

    if args.integers == -1:
        cam = VideoStream(usePiCamera=False).start()
    else:
        cam = VideoStream(src=args.integers).start()
    
    # initialize ros subscriber
    # rospy.Subscriber("/rasendriya/vision_flag", SetBool, vision_flag_callback)
    rospy.Service('/rasendriya/vision_flag', SetBool, vision_flag_req)

    # set lower and upper hsv threshold in red
    lower = np.array([0, 0, 178], dtype='uint8')
    upper = np.array([179, 255, 255],  dtype='uint8')

    while not rospy.is_shutdown():
        
        rospy.loginfo_once("Vision program ready")

        if (vision_flag):
            rospy.loginfo_once("Starting target detection")
            # pre process
            img = cam.read()
            img = imutils.resize(img, width=_width)
            #img_disp = img.copy()
            
            blur = cv2.GaussianBlur(img, (7, 7), 0)

            # color filtering
            frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            frame = cv2.inRange(frame, lower, upper)
            frame = cv2.bitwise_and(blur, blur, mask=frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # circle detection using hough transform
            circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=1.5, minDist=44,
                param1=148, param2=32, #23
                minRadius=0, maxRadius=_width)

            largest_circle_radius = 0
            largest_circle_center = None

            # display bounding circle
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    center = (i[0], i[1])

                    # circle outline
                    radius = (i[2])

                    if radius > largest_circle_radius:
                        largest_circle_radius = radius
                        largest_circle_center = center

            # transform pixel coordinate system to screen coordinate system
            if largest_circle_center is not None:
                x = int(largest_circle_center[0] - _width/2)
                y = int(height/2 - largest_circle_center[1])
                hit_count = hit_count + 1

        if (hit_count > 2):
            dropzone_service_client(x,y)
            hit_count = 0
        
        rate.sleep()

if __name__ == "__main__":
    try:
        dropzone_detect()
    except rospy.ROSInterruptException:
            pass
