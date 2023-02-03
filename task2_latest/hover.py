import time
import cv2
import cv2.aruco as aruco
import numpy as np
from realsense_depth import *
# import the necessary packages
from threading import Thread
from utlities import *
from control_class import *

## IMAGE RESOLUTION
WIDTH = 1280
HEIGHT = 720

def drone_pose(frame ,matrix_coefficients, distortion_coefficients,id_no):
    ## MARKER SIZE OF DRONE
    MARKER_SIZE = 0.056
    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[id_no], MARKER_SIZE, matrix_coefficients,distortion_coefficients)
    return frame,tvec,rvec

# defining an empty custom dictionary 
arucoDict = cv2.aruco.custom_dictionary(0, 4, 1)
# adding empty bytesList array to fill with 5 markers 
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

calibration_matrix_path = "calibration_matrix.npy"
distortion_coefficients_path = "distortion_coefficients.npy"
    
k = np.load(calibration_matrix_path)
d = np.load(distortion_coefficients_path)

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

time.sleep(2.0)

# Initialize the video stream
dc = DepthCamera(width=WIDTH, height=HEIGHT)
first_time = True

num_not_detected = 0
eps = 0.1
desired_depth = 2


# PID PARAMETERS
command = Command("192.168.4.1")
## MEAN - HOVERING VALUES
# mean_roll =  1511
# mean_pitch = 1482
# mean_throttle = 1475


error = np.zeros(4)
derr = np.zeros(4)
error_sum = np.zeros(4)
prev_err = np.zeros(4)
## PITCH,ROLL,THROTTLE,YAW
kp = np.array([1,1,30,0])
ki = np.array([0,0,1,0])
kd = np.array([0,0,0.1,0])
mean_vals = np.array([1500,1500,1470,1500])
# mean_roll =  1510
# mean_pitch = 1513
# mean_throttle = 1470
# mean_yaw = 1500
mean_roll =  1500
mean_pitch = 1500
mean_throttle = 1500
mean_yaw = 1500
slope = 0
for i in range(10):
    command.disarm()
    time.sleep(0.1)
# command.calib()

time.sleep(0.3)
for i in range(10):
    command.disarm()
    time.sleep(0.1)
command.takeoff()
time.sleep(0.2)
prev_time = time.time()
f_old = prev_time
print('[INFO] : Takeoff Completed')
try:
    while True:

        ret, color_frame = dc.get_frame()

        gray = cv2.cvtColor(color_frame,cv2.COLOR_BGR2GRAY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray,arucoDict, parameters=arucoParams)

        if  0 in ids : 
                #print(ids)
                ids = ids.tolist()
                #print('Tracking drone')
                #print(corne)
                i = ids.index([0])

                if ( abs(corners[i][0][1][0] - corners[i][0][0][0]) > 100 or abs(corners[i][0][1][1] - corners[i][0][0][1]) > 100 ) :
                    print(' [ERROR] : Large Marker detected ...')
                
                color_frame,xc,yc = aruco_display(color_frame,corners[i])
                frame,tvec,rvec = drone_pose(color_frame,k,d,i)
                x = tvec[0][0][0]
                y = tvec[0][0][1]

                # point1 = corners[i][0][0] + corners[i][0][3]
                # point2 = corners[i][0][1] + corners[i][0][2]

                point1 = corners[i][0][1] 
                point2 = corners[i][0][3] 
                if point1[0] == point2[0]:
                    slope = 20
                else: 
                    slope =  np.tan((point1[1] - point2[1])/(point1[0] - point2[0]))
                num_not_detected = 0

        else :
            print('[INFO] : Drone not detected ...')
            if num_not_detected > 25:
                print('[INFO] : Drone out of range...')
                print('[INFO] : Landing ...')
                command.land()
                print('[INFO] : Quitting ...')
                break

            num_not_detected +=1
            
            command.set_attitude(mean_throttle,mean_yaw,mean_pitch,mean_roll)
            #time.sleep(0.1)
            continue
        
        ## HOVERING 
        if first_time:
            desired_pos = np.array([x,y,desired_depth,0])
            first_time = False
        else:
            depth = dc.get_depth(xc,yc)
            curr_pos = np.array([x,y,depth,slope])
        
            if(np.linalg.norm(desired_pos - curr_pos) < 0.2):
                print('[INFO] : Reached Position')
                
            print('desired pos : ',desired_pos)
            print('curr_pos :', curr_pos )

            curr_time = time.time()
            time_ = curr_time-prev_time
            error = curr_pos-desired_pos
            derr = (error -prev_err)/(time_)

            P = kp*error
            I = ki*error_sum
            D =  kd*derr
            print(P,I,D)
            result = P+I+D
            print(result)

            # pitch = mean_vals[0] + result[0]
            # roll = mean_vals[1] + result[1]
            # throttle = mean_vals[2] + result[2]
            # yaw = mean_vals[3] + result[3]
            result = mean_vals+ (np.rint(result)).astype(int)
            
            print("pitch : ",result[0])
            print("roll : ",result[1])
            print("throttle : ",result[2])
            print("yaw : ",result[3])
            command.set_attitude(throttle = result[2], yaw = result[3],pitch = result[0],roll = result[1])
            error_sum += error*(time_)
            prev_time = curr_time

            cv2.imshow("Frame", color_frame)

            
        key = cv2.waitKey(1) & 0xFF
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        ## Updating the current time for FPS Calculation
        f_new = time.time()
        fps = 1/(f_new-f_old)
        f_old = f_new
        print('[INFO] : fps = ',fps)

        # Restricting FPS
        # time.sleep(0.01)

except KeyboardInterrupt:
    print('[INFO] : Keyboard Interrupt')
    command.boxarm()
    command.land()
# except:
#     print('[INFO] : Exception')
#     command.boxarm()
#     command.land()

    
dc.release()
cv2.destroyAllWindows()
