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
from math import sqrt

WIDTH = 1280
HEIGHT = 720

def drone_pose(frame ,matrix_coefficients, distortion_coefficients,id_no):
    
    MARKER_SIZE = 0.056
    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[id_no], MARKER_SIZE, matrix_coefficients,distortion_coefficients)
    #frame = aruco_display(frame, corners[i]) 
    return frame,tvec[0][0][0],tvec[0][0][1],tvec[0][0][2]

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
#arucoParams.useAruco3Detection = True
calibration_matrix_path = "calibration_matrix.npy"
distortion_coefficients_path = "distortion_coefficients.npy"
    
k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)
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



Kp_T = 25
Ki_T = 2
Kd_T = 0

I_P = 0
I_T = 0
I_R =0

e_P =0
e_R = 0
e_T = 0

Kp_R = 8
Ki_R = 2
Kd_R = 0

Kp_P = 8
Ki_P = 2
Kd_P = 0

command = Command("192.168.4.1")
for i in range(10):
    command.disarm()
    time.sleep(0.1)
# for i in range(10):
#     command.arm()
#     time.sleep(0.1)
command.takeoff()
# for i in range(20):
#     command.boxarm()
#     print('box arm')
#     time.sleep(0.1)
time.sleep(0.1)

#desired_pos = np.array([420,230,desired_depth])
mean_roll =  1517
mean_pitch = 1482
mean_throttle = 1470

prev_time = time.time()
print('starting off ')
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
                        frame,x,y,z = drone_pose(color_frame,k,d,i)
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
            desired_pos = np.array([x,y,desired_depth])
            first_time = False
        else:
            depth = dc.get_depth(xc,yc)
            # if( depth**2 > x**2 + y**2):
            #     z = sqrt(depth**2 - (x**2 + y**2))
            # else :
            #     print('depth error')
            curr_pos = np.array([x,y,depth])
            #print('depth = ',depth)
        
            if(np.linalg.norm(desired_pos - curr_pos) < eps):
                print('Reached Correct Depth')
                
                # for t in range(50):

                #     command.boxarm()
                #     ret, color_frame = dc.get_frame()
                #     cv2.imshow('Frame',color_frame)        
                #     time.sleep(0.1)
                # #cv2.destroyAllWindows()
                # command.land()
                # print('Landing')
                # break
            print('desired pos : ',desired_pos)
            print('curr_pos :', curr_pos )
            curr_time = time.time()


            print('pitch')
            correction_x,I_P,e_P = PID(Kp_P,Ki_P,Kd_P,curr_pos[0],desired_pos[0],e_P,I_P,curr_time-prev_time)
            print('roll')
            correction_y,I_R,e_R = PID(Kp_R,Ki_R,Kd_R,curr_pos[1],desired_pos[1],e_R,I_R,curr_time-prev_time)
            print('throttle')
            correction_z,I_T,e_T = PID(Kp_T,Ki_T,Kd_T,curr_pos[2],desired_pos[2],e_T,I_T,curr_time-prev_time)


            # command.pitch -= int(correction_x)
            # command.roll += int(correction_y)
            # command.throttle -= int(correction_z)
            command.pitch = mean_pitch + int(correction_x)
            command.roll = mean_roll + int(correction_y) 
            command.throttle = mean_throttle + int(correction_z)
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
        print('fps = ',fps)
        # Restricting FPS
    #time.sleep(0.010)

except KeyboardInterrupt:
    command.boxarm()
    command.land()
except:
    command.boxarm()
    command.land()

    
dc.release()
cv2.destroyAllWindows()
#vs.stop()