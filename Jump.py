def findChessPos(frameCutBuf):
    
    findTime = 1
    chessSumX = 0
    chessSumY = 0
    (m, n, z) = frameCutBuf[0].shape

    #remove color unlike the chess
    for indexPic in range(findTime):
        frameCutChessBuf = np.zeros([m - 60, 150], dtype = np.uint8)
        for indexy in range(30, m - 30):
            for indexx in range(150):
                if (frameCutBuf[indexPic][indexy, indexx, 0] > 50 and 
                    frameCutBuf[indexPic][indexy, indexx, 1] < 100 and
                    frameCutBuf[indexPic][indexy, indexx, 2] < 80):

                    frameCutChessBuf[indexy - 30, indexx] = 255
        #find the small circle head of chess
        frameCutChessBuf = cv2.medianBlur(frameCutChessBuf, 5)
        chessCanny = cv2.Canny(frameCutChessBuf, 80, 100)
        circles = cv2.HoughCircles(chessCanny, cv2.HOUGH_GRADIENT, 1, 100,
                                  param1 = 100, param2 = 16, minRadius = 8, maxRadius = 12)
        cv2.imshow('chessCanny', chessCanny)

        #if not find return 0,0
        if circles is None:
            return (0, 0)
        cirlces = np.uint16(np.around(circles))
        cv2.circle(frameCutBuf[0], (circles[0, 0, 0], (int)(circles[0, 0, 1] + 30)), circles[0, 0, 2], (0, 255, 0), 2)
        
    return ((int)(circles[0, 0, 0] - 54), (int)(circles[0, 0, 1] + 30))


def findNextJumpPos(frameCutBuf, chessX, chessY):
    factor = 0.70
    chessY *= factor

    #resize the pic to make the ellipse to circle
    (h, w, d) = frameCutBuf[0].shape
    h = (int)(h * factor)

    #calculate the position of chess after resize
    chessXo = chessX
    chessYo = chessY

    if chessY < (h / 2) + 10:
        chessX += 10
        chessY += 10
        yzero = (int)((chessY + (1.22 * chessX)))
        chessPos = 0
    else:
        chessX += 10
        chessY -= 10
        yzero = (int)((chessY + (-1.22 * chessX)))
        chessPos = 1
    #find the edge combine by each color channel
    circleCenter = []
    lineCenter = []
    for index in range(10):
        frameResize = cv2.resize(frameCutBuf[index], (w, h))

        b = frameResize[:, :, 0]
        g = frameResize[:, :, 1]
        r = frameResize[:, :, 2]
        
        b = cv2.medianBlur(b, 7)
        g = cv2.medianBlur(g, 7)
        r = cv2.medianBlur(r, 7)

        cannyT1 = 50 
        cannyT2 = 80

        bC = cv2.Canny(b, cannyT1, cannyT2)
        gC = cv2.Canny(g, cannyT1, cannyT2)
        rC = cv2.Canny(r, cannyT1, cannyT2)

        #find the most right point of the image
        frameCanny = bC | gC | rC;
        maxRightP = np.zeros((2), dtype = np.uint8) 
        for i in range(10, (int)(chessYo - 10)):
            for j in range(w):
                if frameCanny[i, w - j - 1] != 0:
                    if (w - j) > maxRightP[0]:
                        maxRightP[0] = w - j
                        maxRightP[1] = i
                        break
        for i in range((int)(chessYo + 10), h - 10):
            for j in range(w):
                if frameCanny[i, w - j - 1] != 0:
                    if (w - j) > maxRightP[0]:
                        maxRightP[0] = w - j
                        maxRightP[1] = i
                        break
        
        cv2.circle(frameResize, (maxRightP[0], maxRightP[1]), 10, (0, 255, 0), 2)

        sumx = 0
        sumy = 0
        #remove left side of the chess to reduce noise
        if chessPos == 0:
            for indexy in range(0 , yzero):
                xline = (int)(((chessY + 1.22 * chessX) - indexy) / 1.22)
                frameCanny[indexy, 0 : xline] = 0
        else:
            for indexy in range(yzero, h):
                xline = (int)(((chessY - 1.22 * chessX) - indexy) / -1.22)
                frameCanny[indexy, 0 : xline] = 0
        #houghcircle
        circles = cv2.HoughCircles(frameCanny, cv2.HOUGH_GRADIENT, 1, 100,
                                  param1 = 100, param2 = 21, minRadius = 10, maxRadius = 43)
        
        if circles is not None:
            cirlces = np.uint16(np.around(circles))
            for i in range(len(circles[0])):
                #append the circle which has distance with the most right point less than 35p
                p = np.array((circles[0, i, 0], circles[0, i, 1]))
                if np.linalg.norm(p - maxRightP) < 43:
                    cv2.circle(frameResize, (circles[0, i, 0], circles[0, i ,1]), circles[0, i, 2], (0, 255, 0), 2)
                    circleCenter.append([(int)(circles[0, i, 0] + 5), circles[0, i ,1]])
                    break;

        #houghline
        lines = cv2.HoughLinesP(frameCanny, 1, np.pi / 180, 30, minLineLength = 25, maxLineGap = 5)
        linesRight = []
        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                #select the line we want
                p1 = np.array((l[0], l[1]))
                p2 = np.array((l[2], l[3]))
                if (np.linalg.norm(p1 - maxRightP) < 10):
                    linesRight.append([l, p2, p1])
                elif (np.linalg.norm(p2 - maxRightP) < 10):
                    linesRight.append([l, p1, p2])
                            
        point = [[0, 0], 0.0]
        if len(linesRight) >= 1:
            for i in range(len(linesRight)):
                p = linesRight[i][1] - linesRight[i][2]
                ang = np.arctan2(p[1], p[0])
                if ((ang <= 2.325) and (ang >= 2.075)):
                    cv2.line(frameResize, (linesRight[i][0][0], linesRight[i][0][1]), (linesRight[i][0][2], linesRight[i][0][3]), (0, 255, 0), 2)
                    point[0][0] = linesRight[i][1]
                    point[0][1] = ang 
        if point[0][1] != 0:
            lenRefLine = np.linalg.norm(point[0][0] - maxRightP)
            deltaY = lenRefLine * np.sin(point[0][1])
            pTmp = (point[0][0][0], (int)(point[0][0][1] - deltaY))
            lineCenter.append(pTmp)

    cv2.imshow('frameCanny', frameCanny)
    cv2.imshow('frameResize', frameResize)
                   

    NextPosX = 0
    NextPosY = 0
    #if NextJump position from houghline has small variance, we can use it
    if len(lineCenter) > 0:
        var = np.var(lineCenter, axis = 0)
        mean = np.mean(lineCenter, axis = 0)
        if var[0] < 100:
            NextPosX = (int)(mean[0])
            NextPosY = (int)(mean[1])
            return (NextPosX, (int)(NextPosY / factor));
        else:
            print('Line var too big')
            print(var)
            
    #elif NextJump position from houghline has small variance, we can use it
    print('circles num', len(circleCenter))

    if len(circleCenter) >= 2:
        
        var = np.var(circleCenter, axis = 0)
        mean = np.mean(circleCenter, axis = 0)
        if var[0] < 20:
            NextPosX = (int)(mean[0])
            NextPosY = (int)(mean[1])
            return (NextPosX, (int)(NextPosY / factor));
        else:
            print('circle var too big')
            print(var)




    return (0, 0)


import numpy as np
import cv2
import copy
import serial.tools.list_ports
import time

print('Hellow')
cap = cv2.VideoCapture(-1)
print(cap.isOpened())
plist = list(serial.tools.list_ports.comports())
time.sleep(2)

while(True):
    # Capture frame-by-frame
    #collect 10 frame a time
    frameCutBuf = []
    i = 0
    while(i < 10):
        ret, frame = cap.read()
        if ret:
            frameCutBuf.append(frame[60 : 419, 230 : 429])
            i += 1
    #line the refence line
    cv2.line(frame, (0, 60), (639, 60), (0, 255, 0), 2)
    cv2.line(frame, (0, 420), (639, 420), (0, 255, 0), 2)

    #chess position 
    chessX, chessY = findChessPos(frameCutBuf)
    cv2.circle(frame, (chessX + 230, chessY + 60), 3, (0, 255, 0), 5)

    #NextJump position
    NextPosX, NextPosY = findNextJumpPos(frameCutBuf, chessX, chessY)
    
    cv2.circle(frame, (NextPosX + 230, NextPosY + 60), 3, (0, 0, 255), 5)
    cv2.imshow('frame', frame)
    cv2.imshow('frameCut', frameCutBuf[0])
    #chess position or NextJump position is invaild
    if ((chessX == 0) or (chessY == 0) or (NextPosX == 0) or (NextPosY == 0)):
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #calculate the distance
    p1 = np.array([chessX + 230, chessY + 60])
    p2 = np.array([NextPosX + 230, NextPosY + 60])
    dis = (int)(np.linalg.norm(p1 - p2))
    
    print('dis=',dis)

    k = (75 / 175)

    b = bytearray(b'0')
    b[0] = (int)(dis * k)
    #send the press time
    print(b[0])
    if len(plist) <= 0:
        print('no com!')
    else:
        with serial.Serial('/dev/ttyUSB0', 115200, timeout = 1) as ser:
            ser.write(b)
    #wait for stabilizing
    for i in range(100):
        ret, frame = cap.read()
        cv2.circle(frame, (chessX + 230, chessY + 60), 3, (0, 255, 0), 5)
        cv2.circle(frame, (NextPosX + 230, NextPosY + 60), 3, (0, 0, 255), 5)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
