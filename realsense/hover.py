#from imutils.video import VideoStream
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from realsense_depth import *
# import the necessary packages
from threading import Thread
from utlities import *
from control_class import *
import math

WIDTH = 1280
HEIGHT = 720


# defining an empty custom dictionary 
# # defining an empty custom dictionary 
arucoDict = cv2.aruco.custom_dictionary(0, 4, 1)
# adding empty bytesList array to fill with 3 markers 
arucoDict.bytesList = np.empty(shape = (5, 2, 4), dtype = np.uint8)

# adding new markers
mybits = np.array([[0,1,0,0],[1,1,0,0],[1,0,1,0],[1,1,0,1]], dtype = np.uint8)
arucoDict.bytesList[0] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[1,1,1,1],[1,0,0,1],[1,0,0,1],[0,0,0,1],], dtype = np.uint8)
arucoDict.bytesList[1] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,0,1],[0,0,0,1],[1,0,1,0],[0,1,1,1]], dtype = np.uint8)
arucoDict.bytesList[2] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,0,0],[1,1,1,0],[1,0,1,1],[0,1,1,1],], dtype = np.uint8)
arucoDict.bytesList[3] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,1,0],[1,0,1,0],[0,0,0,0],[1,1,1,1]], dtype = np.uint8)
arucoDict.bytesList[4] = cv2.aruco.Dictionary_getByteListFromBits(mybits)

arucoParams = aruco.DetectorParameters_create()

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

# Change src accordingly
#vs = WebcamVideoStream(src=6).start()

time.sleep(2.0)

# loop over the frames from the video stream
dc = DepthCamera()
#track_img = vs.read()
first_time = True
xc , yc = -1,-1

num_not_detected = 0
eps = 0.1
#desired_pos = np.array([424,240,1.6])
desired_depth = 1.6

Kp = 30
Ki = 1
Kd = 5

Kp_T = 30
Ki_T = 1
Kd_T = 5

I_P = 0
I_T = 0
I_R =0

e_P =0
e_R = 0
e_T = 0

Kp_R = 1
Ki_R = 0
Kd_R = 0

Kp_P = 1
Ki_P = 0
Kd_P = 0

command = Command("192.168.4.1")
for i in range(10):
    command.disarm()
    time.sleep(0.1)
# for i in range(10):
#     command.arm()
#     time.sleep(0.1)
command.takeoff()
for i in range(20):
    command.boxarm()
    time.sleep(0.1)


command.pitch = 1500
command.roll = 1500
command.throttle = 1500

prev_time = time.time()
try:
    while True:

        ret, color_frame = dc.get_frame()
        detected_markers = color_frame

        (corners, ids, rejected) = cv2.aruco.detectMarkers(color_frame,arucoDict, parameters=arucoParams)

        if len(corners)>0 and 0 in ids : 
                #print('Tracking drone')
                for i in range(0,len(ids)):
                    
                    if ( abs(corners[i][0][1][0] - corners[i][0][0][0]) > 100 or abs(corners[i][0][1][1] - corners[i][0][0][1]) > 100 ) :
                        print('error')
                    if ids[i] == 0:
                        detected_markers,xc,yc = aruco_display(color_frame,corners[i])
                    #cv2.imwrite('det.jpg',  detected_markers)
                    num_not_detected = 0
        else :
            print('not detected')
            if num_not_detected > 10:
                print('Drone out of range')
                command.land()
                break
            num_not_detected +=1
            command.boxarm()
            time.sleep(0.1)
            continue
        
        if first_time:
            desired_pos = np.array([xc,yc,desired_depth])
            first_time = False
            
        if not first_time:
            depth = dc.get_depth(xc,yc)
            curr_pos = np.array([xc,yc,depth])
            print('depth = ',depth)
        
            if(np.linalg.norm(desired_pos-curr_pos) < eps):
                print('Reached Correct Depth')
                
                for t in range(50):

                    command.boxarm()
                    ret, color_frame = dc.get_frame()
                    cv2.imshow('Frame',color_frame)        
                    time.sleep(0.1)
                #cv2.destroyAllWindows()
                command.land()
                print('Landing')
                break
            print('desired pos : ',desired_pos)
            print('curr_pos :', curr_pos )
            curr_time = time.time()


            print('pitch')
            correction_x,I_P,e_P = PID(Kp_P,Ki_P,Kd_P,curr_pos[0],desired_pos[0],e_P,I_P,curr_time-prev_time)
            print('roll')
            correction_y,I_R,e_R = PID(Kp_R,Ki_R,Kd_R,curr_pos[1],desired_pos[1],e_R,I_R,curr_time-prev_time)
            print('throttle')
            correction_z,I_T,e_T = PID(Kp_T,Ki_T,Kd_T,curr_pos[2],desired_pos[2],e_T,I_T,curr_time-prev_time)

  
            command.pitch += int(correction_x)
            command.roll += int(correction_y)
            command.throttle += int(correction_z)
            command.throttle = max(900,min(command.throttle,2100))
            command.pitch = max(900,min(command.pitch,2100))
            command.roll = max(900,min(command.roll,2100))
            print(command.pitch,command.roll,command.throttle)
            command.send()
            cv2.imshow("Frame", detected_markers)

            
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        curr_time = time.time()
        fps = 1/(curr_time-prev_time)
        prev_time = curr_time
        #print('fps = ',fps)
        # Restricting FPS
    #time.sleep(0.010)

except KeyboardInterrupt:
    command.land()

    
dc.release()
cv2.destroyAllWindows()
#vs.stop()