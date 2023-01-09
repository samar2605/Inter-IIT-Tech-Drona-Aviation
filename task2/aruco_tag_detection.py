# Author - Tejadith
import cv2
import cv2.aruco as aruco
import time

aruco_dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)

vid = cv2.VideoCapture(0)
prev_time = time.time()
while True :
    frame = vid.read()[1]
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    (corners, ids, rejected) = aruco.detectMarkers(frame_gray, aruco_dictionary)
    print(corners)
    #frame = aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('Aruuucooo', frame)
    curr_time = time.time()
    fps = 1/(curr_time-prev_time)
    prev_time = curr_time
    print('fps = ',fps)
    
    if cv2.waitKey(1) and 0xFF == ord('q'):
        break

vid.release()
cv2.destroyAllWindows()