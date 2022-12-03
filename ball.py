# -*- coding: utf-8 -*-
"""
Created on Sat Nov 26 2022

@author: MotoWiZ
"""

#%%
import cv2
import time
import numpy as np
import math
from picamera2 import Picamera2
import os
# -> Start pigpio library from prompt. This instruction is needed for the pigpio library to be self-set.
os.system("sudo pigpiod")
time.sleep(0.5)

import pigpio
pi = pigpio.pi() 

# Limits to color matching
lower_white =  (0, 0, 160) #(0, 0, 0) #
upper_white =  (255, 255, 255) #(0, 0, 255) #
# Initialization of a timer to use during the run
t = time.time()
# Attrib values to ball position in the begining so that variables are declared
lastprevBallPosX, lastprevBallPosY, prevBallPosX, prevBallPosY, ballPosX, ballPosY = 0, 0, 0, 0, 0, 0
# Declaration of different PID valriables that are going to be used and need to be initialized
sumErrorX = 0
sumErrorY = 0
maxIx, minIx, maxIy, minIy = 0,0,0,0
alpha, beta, prevAlpha, prevBeta = 0,0,0,0
omega = 0
centerX, centerY = 300,300

# -> Servos configuration for the pigpio library. Values are PIN's and median PWM values for servos
servo1 = 12
servo2 = 13
pwm1 = 910
pwm2 = 1340


#----------------------------------------#
#-----------------CAMERA-----------------#
# -> Configuration of Raspberry Pi camera (Legacy disabled in raspi-config)
def camera_config():
    picam2 = Picamera2() 
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800, 600)}))
    picam2.start()
    return picam2
#-------------END OF CAMERA--------------#
#----------------------------------------#



#----------------------------------------#
#-----------------ROI--------------------#
# -> Configure ROI with double mouse clicks
def mark_roi(event,x,y,flags,param):
  global mouseX, mouseY, nr_click
  if event == cv2.EVENT_LBUTTONDBLCLK:
      cv2.circle(img,(x,y),10,(0, 0, 255),-1)
      mouseX, mouseY = x, y
      nr_click += 1
#--------------END OF ROI----------------#
#----------------------------------------#


#----------------------------------------#
#----------------IMAGING-----------------#
# -> Crop and convert image from camera    
def get_image(picam2):
    img = picam2.capture_array()
    img = cv2.cvtColor(img,cv2.COLOR_RGBA2RGB)
    return img
#-------------END OF IMAGING-------------#
#----------------------------------------#



#----------------------------------------#
#--------------BALL FINDER---------------#
# -> Find white ball on table
def find_ball(img, x_cent, y_cent):
    ball_exists = False
    cX, cY = 0, 0
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    # Mask image for whites
    mask = cv2.inRange(hsv, lower_white, upper_white)
    # Get bitwise image with mask    
    result = cv2.bitwise_and(img, img, mask = mask)
    # Transform img in Black & White
    gray = cv2.cvtColor(result,cv2.COLOR_RGB2GRAY)
    (thresh, bw) = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    # Detect if exeists white object
    if np.sum(bw) > 50000:
        # Find center of white ball   
        M = cv2.moments(mask)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        # Place red circle and text in center of white ball
        cv2.circle(result, (cX, cY), 10, (0, 0, 255), -1)
        cv2.putText(result, "Object center", (cX - 110, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        ball_exists = True
    else:
        cv2.putText(result, "No ball detected", (x_cent - 130, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(img, "No ball detected", (x_cent - 130, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    # Place green circle in center of result
    cv2.circle(result, (x_cent, y_cent), 10, (0, 255, 0), -1)
    # Place green circle in center of reduced image
    cv2.circle(img, (x_cent, y_cent), 10, (0, 255, 0), -1)
    return img, result, ball_exists, cX, cY
#-----------END OF BALL FINDER-----------#
#----------------------------------------#

"""
        NOTES: 
        In our case the servos should only move to PWM:
        SERVO 1 PWM: MAX -> 1150 mean -> 910 min -> 670
        SERVO 2 PWM: MAX -> 1580 mean -> 1340 min -> 1100
        (NOTE: servos are inverted so in SERVO 1 MAX and min
        correspond to top and bottom, but for SERVO 2 MAX and min correspond to bottom and top, respectively)
        For the calculated angles we consider the following
        Alpha - Affects X axis ==> Servo 2
        Beta - Affects Y axis ==> Servo 1


        _summary_: This is our table schem:
                                                        SERVO 1
                                                          90º
                                             _____________________________
                                            |          *** |***           | 
                                            |        ***   |   ***        |
                                            | 135º ***     |     *** 45º  |
                                            |    ***       |       ***    |         
                                            |  ***         |         ***  |                                         
                               SERVO 2 180º |***-----------|-----------***| 0º/360º                                                       
                                            |  ***         |         ***  |                                            
                                            |    ***       |       ***    |                                             
                                            | 225º ***     |     *** 315º |                                             
                                            |        ***   |   ***        |                                            
                                            |          *** | ***          |                                             
                                            |_____________________________|      
                                                          270º                                                                                                                                                                                
"""     
    
#----------------------------------------#
#------------PID CONTROL-----------------#
def PIDcontrol(ballPosX, ballPosY, lastprevBallPosX, lastprevBallPosY, prevBallPosX, prevBallPosY, centerX, centerY):
    global omega
    global sumErrorX, sumErrorY
    global maxIx, minIx, maxIy, minIy
    global alpha, beta
    # Parameters definition:
    Kp = 0.65 #valor de Kp 0.65
    Ki = 0.015 #valor de Ki 0.015
    Kd = 0.5 #valor de Kd 0.5
    Ka = 0.3 #valor de Ka 0.3
    
    # Calculate accelaration
    accX = (prevBallPosX - ballPosX) - (lastprevBallPosX - prevBallPosX) 
    accY = (prevBallPosY - ballPosY) - (lastprevBallPosY - prevBallPosY)
    # Calculate errors, to be utilized in PID:
    sumErrorX += centerX - ballPosX
    sumErrorY += centerY - ballPosY
	# Error calculation in X and Y (PID):
    if abs(centerX-ballPosX) < abs(centerX-prevBallPosX) and abs(centerX-prevBallPosX) < abs(centerX-lastprevBallPosX):
        Ix = 0
        sumErrorX = 0
    else:
        Ix = Kp*(centerX-ballPosX) + Ki*(sumErrorX) + Ka*accX + Kd*(prevBallPosX-ballPosX)
    if abs(centerY-ballPosY) < abs(centerY-prevBallPosY) and abs(centerY-prevBallPosY) < abs(centerY-lastprevBallPosY):
        Iy = 0
        sumErrorY = 0
    else:
        Iy = Kp*(centerY-ballPosY) + Ki*(sumErrorY) + Ka*accY + Kd*(prevBallPosY-ballPosY)
    # Round errors to 4 decimal
    Ix = round(Ix/1, 4)
    Iy = round(Iy/1, 4)
    # Calculate max and min de Ix and Iy
    if Ix > maxIx:
        maxIx = Ix
    elif Ix < minIx:
        minIx = Ix
    if Iy > maxIy:
        maxIy = Iy
    elif Iy < minIy:
        minIy = Iy
    
    # Store preveous PID Values if needed in future   
    prevAlpha = alpha
    prevBeta = beta
    alpha = Ix #0 # 
    beta = Iy #0 # 
    print("PID max & min values: ", maxIx, minIx, maxIy, minIy)
    print("PID values: ", Ix, Iy, math.sqrt(Ix**2 + Iy**2))
    print("Center & BallPos:", centerX, centerY, ballPosX, ballPosY, prevBallPosX, prevBallPosY)  
    print("Error X & Y: ", sumErrorX, sumErrorY)
    #print("Center & BallPos:", centerX, centerY, ballPosX, ballPosY)
    #print("BallPos:", abs(centerX - ballPosX), abs(centerY - ballPosY)) 
    return alpha, beta
#----------END OF PID CONTROL------------#
#----------------------------------------#


#----------------------------------------#
#-----------------SERVOS-----------------#
# -> Changing the Duty Cycle to rotate the Servos
def servos(alpha, beta):
    pwm1 = 910  # PWM median value, table with no inclination in Y
    pwm2 = 1340 # PWM median value, table with no inclination in X
    # Affect PWM's of the ammounts from PID
    pwm1 = pwm1 + beta
    pwm2 = pwm2 - alpha
    # Min and max values for pwm's
    if pwm1 < 670:
        pwm1 = 670 # 
    elif pwm1 > 1150:
        pwm1 = 1150
    if pwm2 < 1100:
        pwm2 = 1100
    elif pwm2 > 1580:
        pwm2 = 1580
    #print("PID & PWM: ", alpha, beta, pwm1, pwm2)
    print("PWM: ", pwm1, pwm2)
    return pwm1, pwm2
#------------END OF SERVOS---------------#
#----------------------------------------#


#----------------------------------------#
#---------------IMAGE WINDOWS------------#
# -> Windows for original, reduced and mask images
def windows(original, img, result):
    # Original image window
    cv2.namedWindow("Original image", cv2.WINDOW_NORMAL)
    cv2.imshow("Original image",original)
    cv2.resizeWindow("Original image", 400, 300)
    cv2.moveWindow("Original image", 10,100)
    # Reduced image window
    cv2.namedWindow("Reduced image", cv2.WINDOW_NORMAL)
    cv2.imshow("Reduced image",img)
    cv2.resizeWindow("Reduced image", 400, 300)
    cv2.moveWindow("Reduced image", 440,100)
    # Masked image window
    cv2.namedWindow("Masked image", cv2.WINDOW_NORMAL)
    cv2.imshow("Masked image",result)
    cv2.resizeWindow("Masked image", 400, 300) 
    cv2.moveWindow("Masked image", 870,100) 
#---------END OF IMAGE WINDOWS-----------#
#----------------------------------------#
    
    
#----------------------------------------#
#---------------MAIN PROGRAM-------------#
# -> Main
if __name__ == "__main__":
    picam2 = camera_config()
    img = get_image(picam2)
    cv2.namedWindow("Pick points")
    cv2.moveWindow("Pick points", 280,100) 
    cv2.setMouseCallback("Pick points",mark_roi)
    nr_click = 0
    prev_nr_click = nr_click
    x_pos, y_pos = [], []
        
    # -> Start picker of corners
    while True:
        cv2.imshow("Pick points",img)
        k = cv2.waitKey(1) & 0xFF
        if nr_click == 3 or k == ord('q'):
            break
        elif nr_click != prev_nr_click:
            prev_nr_click = nr_click
            x_pos.append(mouseX)
            y_pos.append(mouseY)
            #print (mouseX,mouseY,nr_click)
    cv2.destroyAllWindows()
    #print (x_pos[0], y_pos[0])
    #print (x_pos[1], y_pos[1])
    
    # -> Find ball during movement
    while True:
        # Start timer to count FPS
        start_time = time.time()
        original = get_image(picam2)
        img = original.copy()
        # Calculate center of croped image and reduce original image
        if y_pos[0] < y_pos[1]:
            y_cent = int((y_pos[1] - y_pos[0])/2)
            if x_pos[0] < x_pos[1]:
                img = img[y_pos[0]:y_pos[1], x_pos[0]:x_pos[1]]
                x_cent = int((x_pos[1] - x_pos[0])/2)
            elif x_pos[0] > x_pos[1]:
                img = img[y_pos[0]:y_pos[1], x_pos[1]:x_pos[0]]
                x_cent = int((x_pos[0] - x_pos[1])/2)
        elif y_pos[0] > y_pos[1]:
            y_cent = int((y_pos[0] - y_pos[1])/2)
            if x_pos[0] < x_pos[1]:
                img = img[y_pos[1]:y_pos[0], x_pos[0]:x_pos[1]]
                x_cent = int((x_pos[1] - x_pos[0])/2)
            elif x_pos[0] > x_pos[1]:
                img = img[y_pos[1]:y_pos[0], x_pos[1]:x_pos[0]]
                x_cent = int((x_pos[0] - x_pos[1])/2)
        #print(x_cent, y_cent)
        if ballPosX != 0:
            lastprevBallPosX = prevBallPosX
            lastprevBallPosY = prevBallPosY
            prevBallPosX = ballPosX
            prevBallPosY = ballPosY
        img, result, ball_exists, ballPosX, ballPosY  = find_ball(img, x_cent, y_cent)
        # Call PID function
        if ball_exists:
            alpha, beta = PIDcontrol(ballPosX, ballPosY, lastprevBallPosX, lastprevBallPosY, prevBallPosX, prevBallPosY, x_cent, y_cent)
            pwm1, pwm2 = servos(alpha, beta)
            #if pwm1 != 910 or pwm2 != 1340:
            pi.set_servo_pulsewidth(servo1, pwm1)
            pi.set_servo_pulsewidth(servo2, pwm2)
        else:
            pi.set_servo_pulsewidth(servo1, 910)
            pi.set_servo_pulsewidth(servo2, 1340)
            sumErrorX = 0
            sumErrorY = 0
            
        #print(pwm1, pwm2)
        # Place green circle in center of original image
        cv2.circle(original, (400, 300), 10, (0, 255, 0), -1)
        windows(original, img, result)        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
        # Print Frames per Second
        if time.time() - t > 2:
            print("FPS: ", "%.2f" % (1.0 / (time.time() - start_time)))
            t = time.time()
    # Exit closing cv2 windows
    os.system("sudo killall pigpiod")
    cv2.destroyAllWindows()
