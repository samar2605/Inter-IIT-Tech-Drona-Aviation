#from imutils.video import VideoStream
import imutils
import time
import cv2
import cv2.aruco as aruco
import numpy as np
from realsense_depth import *
# import the necessary packages
from threading import Thread

WIDTH = 1280
HEIGHT = 720


def aruco_display(corners, ids, rejected, color_frame,tracking_frame):
    # verify *at least* one ArUco marker was detected
    cX = 0
    cY = 0

    print(ids)
    if len(corners) > 0:
        #print('ArUco Tag detected')
        # flatten the ArUco IDs list
        print(ids)
        ids = ids.flatten()
        
        # loop over the detected ArUco corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(color_frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(color_frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(color_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(color_frame, bottomLeft, topLeft, (0, 255, 0), 2)



            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(color_frame, (cX, cY), 1, (0, 0, 255), -1)
            cv2.circle(tracking_frame, (cX, cY), 1, (0, 0, 255), -1)
            # draw the ArUco marker ID on the frame
            cv2.putText(color_frame, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))	
    return (color_frame,tracking_frame,cX,cY)


# defining an empty custom dictionary 
arucoDict = cv2.aruco.custom_dictionary(0, 4, 1)

# adding empty bytesList array to fill with 3 markers 
arucoDict.bytesList = np.empty(shape = (3, 2, 4), dtype = np.uint8)

# adding new markers
mybits = np.array([[0,1,0,0],[1,1,0,0],[1,0,1,0],[1,1,0,1]], dtype = np.uint8)
arucoDict.bytesList[0] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,1],], dtype = np.uint8)
arucoDict.bytesList[1] = cv2.aruco.Dictionary_getByteListFromBits(mybits)
mybits = np.array([[0,1,1,1],[1,0,0,0],[1,0,0,0],[1,1,1,1]], dtype = np.uint8)
arucoDict.bytesList[2] = cv2.aruco.Dictionary_getByteListFromBits(mybits)

arucoParams = aruco.DetectorParameters_create()

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

# Change src accordingly
#vs = WebcamVideoStream(src=6).start()

time.sleep(2.0)
prev_time = time.time()
# loop over the frames from the video stream
dc = DepthCamera()
#track_img = vs.read()
track_img = None
first_time = True
while True:
    #frame = vs.read()
    #print('test')
    ret, depth_frame, color_frame = dc.get_frame()
    if first_time:
        track_img = color_frame
        track_img = 255 + 0*track_img
        
        first_time = False
    #cv2.imshow("Tracking Frame p", track_img)
    #frame = imutils.resize(frame, width=720,height = 480)
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(color_frame,arucoDict, parameters=arucoParams)
   # print(track_img)
    
    #track_img = aruco_display_minimal(corners, ids, rejected, track_img)
        #print(track_img)
  #  print(track_img)
    
    # frame = aruco.drawDetectedMarkers(frame, corners, ids)  # this can also be used but didn't pinpoint the center
    # cv2.imshow("Frame",frame)
    x_c , y_c = 100,100
    # show the output frame
    detected_markers,track_img,x_c,y_c = aruco_display(corners, ids, rejected, color_frame,track_img)
    cv2.imshow("Frame", detected_markers)
    cv2.imshow("Tracking Frame", track_img)
    depth = depth_frame[y_c,x_c]
    print(depth)
    key = cv2.waitKey(1) & 0xFF
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
    curr_time = time.time()
    fps = 1/(curr_time-prev_time)
    prev_time = curr_time
    print('fps = ',fps)

    # Restricting FPS
    time.sleep(0.010)
cv2.imwrite('tracked.jpg',track_img)
cv2.destroyAllWindows()
#vs.stop()