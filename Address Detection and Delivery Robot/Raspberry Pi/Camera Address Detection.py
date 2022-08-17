import cv2
import cv2.aruco as aruco
import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)


cap = cv2.VideoCapture(0)

def Aruco(img, markersize = 6, total = 250, draw = True):
    
    key = getattr(aruco, f'DICT_{markersize}X{markersize}_{total}')
    Dictonary = aruco.Dictionary_get(key)
    Para = aruco.DetectorParameters_create()
    bbox, ids,_ = aruco.detectMarkers(img, Dictonary, parameters = Para)
    print(ids)

    if draw :
        
        aruco.drawDetectedMarkers(img, bbox)

    return bbox, ids

def Servomove(Servoangle):
    pulsew = Servoangle*9.72 + 500
    pulsewidth = pulsew/1000000
    for indx in range(0,2):
        GPIO.output(23,1)
        time.sleep(pulsewidth)
        GPIO.output(23,0)
        time.sleep(0.1)

def roi(img):
    height = img.shape[0]
    height = height + 100
    width = img.shape[1]
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #ROI_up = np.array([[(117, 10), (117, 200), (510, 200), (510, 10)]], dtype = np.int32)
    ROI = np.array([[(130,height),(130,180),(510,180),(510,height)]], dtype = np.int32)
    black = np.zeros_like(grey_img)
    roi = cv2.fillPoly(black, ROI, 255)
    roi_img = cv2.bitwise_and(grey_img,roi)
    return roi_img

count69 = 0
count0 = 0
count1 = 0
count2 = 0
count3 = 0
count4 = 0
count5 = 0
count6 = 0
count7 = 0
count8 = 0
count9 = 0
count10 = 0
count11 = 0
count12 = 0
count13 = 0
count14 = 0
count15 = 0
count16 = 0
count17 = 0
count18 = 0
count19 = 0
count = count10 + count11 + count12 + count13 + count14 + count15 + count16 + count17 + count18 + count19
x = 10
y = 180

while True:
    GPIO.output(17,0)
    _, img = cap.read()
    roi_img = roi(img)
    bbox, ids = Aruco(roi_img)
    tatti = int(ids or 69)
    
    if count69 == 0:
        Servomove(x)
        time.sleep(0.2)
        Servomove(y)
        time.sleep(0.2)
        count69 = count69 + 1

    if tatti == 10:
        count10 = count10 + 1
        if count10 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 
        count11 = count11 + 1
        if count11 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 12:
        count12 = count12 + 1
        if count12 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 13:
        count13 = count13 + 1
        if count13 == 1:
            GPIO.output(17,1)
            time.sleep(0.5)
            GPIO.output(27,1)
            GPIO.output(17,0)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 14:
        count14 = count14 + 1
        if count14 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 15:
        count15 = count15 + 1
        if count15 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
            
    elif tatti == 16:
        count16 = count16 + 1
        if count16 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 17:
        count17 = count17 + 1
        if count17 == 1:
            GPIO.output(17,1)
            time.sleep(0.3)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 18:
        count18 = count18 + 1
        if count18 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 19:
        count19 = count19 + 1
        if count19 == 1:
            GPIO.output(17,1)
            time.sleep(0.3)
            GPIO.output(17,0)
            GPIO.output(27,1)
            time.sleep(0.5)
            GPIO.output(27,0)
            Servomove(x)
            time.sleep(0.2)
            Servomove(y)
            time.sleep(0.2)
    elif tatti == 0:
        count0 = count0 + 1
        if count0 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 1:
        count1 = count1 + 1
        if count1 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 2:
        count2 = count2 + 1
        if count2 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 3:
        count3 = count3 + 1
        if count3 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 4:
        count4 = count4 + 1
        if count4 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 5:
        count5 = count5 + 1
        if count5 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 6:
        count6 = count6 + 1
        if count6 == 1:
            GPIO.output(17,1)
            time.sleep(0.3)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 7:
        count7 = count7 + 1
        if count7 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 8:
        count8 = count8 + 1
        if count8 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    elif tatti == 9:
        count9 = count9 + 1
        if count9 == 1:
            GPIO.output(17,1)
            time.sleep(2)
            GPIO.output(17,0)
            GPIO.output(22,1)
            time.sleep(0.5)
            GPIO.output(22,0)
    
                
        
    if cv2.waitKey(1) == 113:
        break
    cv2.imshow("Live Feed", roi_img)

