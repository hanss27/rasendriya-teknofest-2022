import rospy
from rasendriya.srv import Dropzone 
import cv2
import numpy as np
import imutils
import argparse
import os
from imutils.video import VideoStream
from std_srvs.srv import SetBool, SetBoolResponse

vision_flag = False
vision_flag_old = False
hit_count_thres = 0

# Sending Dropzone Service
def dropzone_service_client(x,y):
    dropzone_service = rospy.ServiceProxy('/rasendriya/dropzone', Dropzone)
    resp = dropzone_service(x,y)
    rospy.loginfo("Dropzone coordinates sent")
    return resp.status

def vision_flag_req(req):
    global vision_flag
    vision_flag = req.data
    return SetBoolResponse(True, "Flag set to on. Scanning")

def draw(_img, _ctr, _rad, _hit_cnt):
    global hit_count_thres
    img_path = os.getcwd()
    # draw and save image
    cv2.circle(_img, _ctr, 1, (255,255,255), 3)
    cv2.putText(_img, "center", (_ctr[0] - 20, _ctr[1] - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # circle outline
    cv2.circle(_img, _ctr, _rad, (0,255,0), 3)
	
    if (_hit_cnt > 7): # Hit Count
        cv2.imwrite("/home/ubuntu/detected.jpg", _img)
    else:
        trgt_img = "scan_{}.jpg".format(str(_hit_cnt))
        cv2.imwrite(os.path.join("/home/ubuntu/", trgt_img), _img)
    rospy.loginfo("Image Written!")

def dropzone_detect():
    global vision_flag, vision_flag_old
    # initialize ros node
    rospy.init_node('vision_dropzone')
    rospy.wait_for_service('/rasendriya/dropzone')

    loop_rate = rospy.get_param('/rasendriya/loop_rate')
    hit_count_thres = rospy.get_param('/rasendriya/hit_count_thres')

    # initialize ros publisher
    rate = rospy.Rate(loop_rate)

    hit_count = 0
    x = -3000
    y = -3000

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

    rospy.sleep(2.)
    
    # initialize ros subscriber
    rospy.Service('/rasendriya/vision_flag', SetBool, vision_flag_req)

    # set lower and upper hsv threshold in red
    lower = np.array([0, 0, 178], dtype='uint8')
    upper = np.array([179, 255, 255],  dtype='uint8')

    while not rospy.is_shutdown():
        
        rospy.loginfo_once("Vision program ready")

        if((vision_flag == True) and (vision_flag_old == False)):
            rospy.loginfo("Starting target scanning")
            
        elif((vision_flag == False) and (vision_flag_old == True)):
            rospy.loginfo("Stopping target scanning")

        if (vision_flag):
            # pre process
            img = cam.read()
            img = imutils.resize(img, width=_width)
            
            blur = cv2.GaussianBlur(img, (5, 5), 0)

            # color filtering
            frame = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
            frame = cv2.inRange(frame, lower, upper)
            frame = cv2.bitwise_and(blur, blur, mask=frame)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # circle detection using hough transform
            circles = cv2.HoughCircles(frame, method=cv2.HOUGH_GRADIENT, dp=1.5, minDist=131,
                param1=196, param2=32, #23
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

                # increase counter
                hit_count = hit_count + 1
                draw(img, center, radius, hit_count)
                
                #rospy.loginfo("x: {}".format(x))
                #rospy.loginfo("y: {}".format(y))

                if (hit_count > 7):
                    draw(img, center, radius, hit_count)
                    dropzone_service_client(x,y)
                    vision_flag = False

            else:
                hit_count = 0
		
        vision_flag_old = vision_flag
        rate.sleep()
        

if __name__ == "__main__":
    try:
        dropzone_detect()
    except rospy.ROSInterruptException:
            pass
