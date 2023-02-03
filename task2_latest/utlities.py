import cv2
def aruco_display(frame,corners):
    # verify *at least* one ArUco marker was detected
    # flatten the ArUco IDs list

    # loop over the detected ArUco corners
        # extract the marker corners (always returned
        # in top-left, top-right, bottom-right, and bottom-left
        # order)
    corners = corners.reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = corners
    # convert each of the (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))

    # draw the bounding box of the ArUCo detection
    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)



    # compute and draw the center (x, y)-coordinates of the
    # ArUco marker
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    cv2.circle(frame, (cX, cY), 1, (0, 0, 255), -1)
    # draw the ArUco marker ID on the frame
    #cv2.putText(frame, str(markerID),(topLeft[0], topLeft[1] - 15),cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
    #print("[Inference] ArUco marker ID: {}".format(markerID))	
    return frame,cX,cY


def PID (kp,ki,kd,PV,SP,e_prev,I,dt):
    e = PV - SP
    P = kp*e
    I += ki*(e)*dt
    if dt != 0:
        D = kd*(e-e_prev)/dt
    else:
        D =0
    
    correction = P+I+D
    print(P,I,D)
    
    return correction,I,e