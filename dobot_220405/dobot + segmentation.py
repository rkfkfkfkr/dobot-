import cv2
import numpy as np
import imutils
import matplotlib.pyplot as plt
import threading
import DobotDllType as dType
import math


cap = cv2.VideoCapture(0)

w = round(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)
fourcc = cv2.VideoWriter_fourcc(*'DIVX')

out = cv2.VideoWriter('following_ball_220404.mp4', fourcc, fps, (w, h))

objectPoints = np.array([[0,0,0],
                        [0,-0.325,0],
                        [0,0.325,0],
                        [0.449,0.325,0],
                        [0.449,-0.325,0],
                        [0.449,0,0]],dtype = 'float32')
imagePoints = np.array([[320,243],
                       [139,243],
                       [478,243],
                       [602,478],
                       [36,478],
                       [320,478]],dtype = 'float32')

fx = float(470.5961)
fy = float(418.18176)
cx = float(275.7626)
cy = float(240.41246)
k1 = float(0.06950)
k2 = float(-0.07445)
p1 = float(-0.01089)
p2 = float(-0.01516)

cameraMatrix = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]],dtype = 'float64')
distCoeffs = np.array([k1,k2,p1,p2],dtype = 'float64')
_,rvec,t = cv2.solvePnP(objectPoints,imagePoints,cameraMatrix,distCoeffs)
R,_ = cv2.Rodrigues(rvec)

CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound", 
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}

#dll을 메모리에 읽어들여 CDLL 인스턴스 가져오기
#Load Dll and get the CDLL object
api = dType.load()

#dobot과의 연결을 설정
#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

def get_distance(R, t, fx, fy, cx, cy, x, y):
    
    u = (x - cx) / fx
    v = (y - cy) / fy
    Qc = np.array([[u],[v],[1]])
    Cc = np.zeros((3,1))
    Rt = np.transpose(R)
    Qw = Rt.dot((Qc-t))
    Cw = Rt.dot((Cc-t))
    V = Qw - Cw
    k = -Cw[-1,0]/V[-1,0]
    Pw = Cw + k*V
    
    return Pw

def cpmove(of_x,of_y):

    of_x = (of_x+4) *10 + 28
    of_y = of_y *10 + 37
    
    l_index = dType.SetCPCmd(api,dType.ContinuousPathMode.CPAbsoluteMode, of_x, of_y, 50, 30, isQueued = 1)

    dType.dSleep(100)

    return l_index

if (state == dType.DobotConnect.DobotConnect_NoError):
    
    #Clean Command Queued
    dType.SetQueuedCmdClear(api)
    
    #Async Motion Params Setting
    dType.SetHOMEParams(api, 200, 0, 50, 100, isQueued = 1) # x, y, z, r 
    dType.SetCPParams(api, 50, 200, 200, True, isQueued = 1 ) # SetCPParams(api, planAcc, juncitionVel, acc, realTimeTrack = 0,  isQueued=0)
    dType.SetCPCommonParams(api, 30,30, isQueued = 1)
    
    #Async Home
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)

    count = 0

    dType.SetQueuedCmdStartExec(api)

    while(1):
        ret,frame = cap.read()

        img_ycrcb = cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
        y,cr,cb = cv2.split(img_ycrcb)

        _, cb_th = cv2.threshold(cb, 100, 255, cv2.THRESH_BINARY_INV)
        cb_th = cv2.erode(cb_th, None, iterations=2)
        cb_th = cv2.dilate(cb_th, None, iterations=2)

        cnts = cv2.findContours(cb_th, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 5:
                cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                
                Pw = get_distance(R,t,fx,fy,cx,cy, center[0], center[1])
                px = 100 * Pw[0]
                py = 100 * Pw[1]
                text = " %f , %f" %(px,py)
                cv2.putText(frame,text,center,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)
                
                offset_x = px
                offset_y = py
                
                if count == 0:
                    current_ofx = px
                    current_ofy = py

                    last_index = cpmove(offset_x,offset_y)                    

                else:
                    x_move = abs(current_ofx - px)
                    y_move = abs(current_ofy - py)

                    if last_index == dType.GetQueuedCmdCurrentIndex(api)[0]:
                            dType.SetQueuedCmdStartExec(api)
                            pirnt("a")

                    if 2 < x_move or 2 < y_move:

                            last_index = cpmove(offset_x,offset_y)
                                    
                #if count > 0:
                    #last_index == dType.GetQueuedCmdCurrentIndex(api)[0]

                    #dType.SetQueuedCmdStopExec(api)
                        
                
                count += 1
                print(last_index)
                print(dType.GetQueuedCmdCurrentIndex(api)[0])
                pose1 = dType.GetPose(api)
                text2 = " %f , %f" %(pose1[0],pose1[1])
                cv2.putText(frame,text2,(0,20),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2)

                
        cv2.imshow('cam',frame)
        out.write(frame)

        if cv2.waitKey(1) == 27:
            break

    dType.SetQueuedCmdStopExec(api)
    

dType.DisconnectDobot(api)
cap.release()
out.release()
cv2.destroyAllWindows()
