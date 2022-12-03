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
import RPi.GPIO as GPIO
from picamera2 import Picamera2 

lower_white = (0, 0, 0)
upper_white = (0, 0, 255)
t = time.time()
prevBallPosX, prevBallPosY, ballPosX, ballPosY = 0, 0, 0, 0

#-> Servos configuration
SERVO_PIN1 = 32
SERVO_PIN2 = 33
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN1,GPIO.OUT)
GPIO.setup(SERVO_PIN2,GPIO.OUT)
# Disable the warning from the GPIO Library
GPIO.setwarnings(False)
# Starting the PWM and setting the initial position of the servo with 50Hz frequency 
servo1 = GPIO.PWM(SERVO_PIN1,50)
servo1.start(0)
servo2 = GPIO.PWM(SERVO_PIN2,50)
servo2.start(0)

# -> Configuration of Raspberry Pi camera (Legacy disabled in raspi-config)
def camera_config():
    picam2 = Picamera2() 
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800, 600)}))
    picam2.start()
    return picam2

# -> Configure ROI with double mouse clicks
def mark_roi(event,x,y,flags,param):
  global mouseX, mouseY, nr_click
  if event == cv2.EVENT_LBUTTONDBLCLK:
      cv2.circle(img,(x,y),10,(0, 0, 255),-1)
      mouseX, mouseY = x, y
      nr_click += 1

# -> Crop and convert image from camera    
def get_image(picam2):
    img = picam2.capture_array()
    img = cv2.cvtColor(img,cv2.COLOR_RGBA2RGB)
    return img

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
    
# Changing the Duty Cycle to rotate the Servos
def servos(alpha, beta):
    pwm1 = 4.40
    pwm2 = 6.25
    """
            NOTE: In our case the servos should only move to PWM:
            SERVO 1: MAX -> 5.5 min -> 3.3
            SERVO 2: MAX -> 7.5 min -> 5
    """
    inclin1 = round((alpha - 0.3), 2)
    inclin2 = round((alpha - 0.15), 2)
    
    
    # First quadrant
    if beta == 0 or beta == 360:
        pwm2 = pwm2 + inclin2
    elif beta > 0 and beta < 90:
        coef_beta = round((beta / 45) , 2)
        if coef_beta < 1:
            pwm1 = round(pwm1 + (inclin1 * (coef_beta)), 2)
            pwm2 = round((pwm2 + inclin2), 2)
        if coef_beta > 1:
            pwm1 = round((pwm1 + inclin1), 2)
            pwm2 = round(pwm2 + (inclin2 * (1-(coef_beta-1))), 2)
    # Second quadrant
    elif beta == 90:
        pwm1 = pwm1 + inclin1
    elif beta > 90 and beta < 180:
        coef_beta = round(((beta-90) / 45) , 2)
        if coef_beta < 1:
            pwm1 = round((pwm1 + inclin1), 2)
            pwm2 = round(pwm2 - (inclin2 * (coef_beta)), 2)
        if coef_beta > 1:
            pwm1 = round(pwm1 + (inclin1 * (1-(coef_beta-1))), 2)
            pwm2 = round((pwm2 - inclin2), 2)
    # Third quadrant
    elif beta == 180:
        pwm2 = pwm2 - inclin2
    elif beta > 180 and beta < 270:
        coef_beta = round(((beta-180) / 45) , 2)
        if coef_beta < 1:
            pwm1 = round(pwm1 - (inclin1 * (coef_beta)), 2)
            pwm2 = round((pwm2 - inclin2), 2)
        if coef_beta > 1:
            pwm1 = round((pwm1 - inclin1), 2)
            pwm2 = round(pwm2 - (inclin2 * (1-(coef_beta-1))), 2)
    # Fourth quadrant
    elif beta == 270:
        pwm1 = pwm1 - inclin1
    elif beta > 270 and beta < 360:
        coef_beta = round(((beta-270) / 45) , 2)
        if coef_beta < 1:
            pwm1 = round((pwm1 - inclin1), 2)
            pwm2 = round(pwm2 + (inclin2 * (coef_beta)), 2)
        if coef_beta > 1:
            pwm1 = round(pwm1 - (inclin1 * (1-(coef_beta-1))), 2)
            pwm2 = round((pwm2 + inclin2), 2)        
    
    # Min and max values for pwm's
    if pwm1 < 3.3:
        pwm1 = 3.3
    elif pwm1 > 5.5:
        pwm1 = 5.5
    if pwm2 < 5:
        pwm2 = 5
    elif pwm2 > 7.5:
        pwm2 = 7.5
        
    print(alpha, beta, pwm1, pwm2)
    return pwm1, pwm2

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
    
    
#----------------------------------------#
#------------PID CONTROL-----------------#
# Nota inicial: Eles aqui utilizam os ângulos alpha e beta. A meu ver, devíamos utilizar estes ângulos de maneira a 
# que alpha controla, por exemplo, o movimento da bola no eixo X e beta controla a bola no eixo Y.
# Os seguintes valores têm que ser inseridos: coordenadas do centro da bandeja, Kp, Ki, Kd, 
sumErrorX = 1
sumErrorY = 1
#timeInterval = 1 <- Creio que não seja necessário
alpha, beta, prevAlpha, prevBeta = 0,0,0,0
omega = 0 #0.2
centerX, centerY = 240,240 # <- CENTRO DA BANDEJA, não sei os valores. Não precisa de ser declarado aqui, deve 
#ser determinado noutro sítio qualquer e deve ser colocado nas variáveis globais centerX e centerY 

def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, centerX, centerY):
    global omega
    global sumErrorX, sumErrorY
    global alpha, beta, prevAlpha, prevBeta
    
	#Definição dos parâmetros:
    Kp = 0.6 #valor de Kp
    Ki = 0 #valor de Ki
    Kd = 0 #valor de Kd
	
	#Cálculo dos erros em X e Y:
    Ix = Kp*(centerX-ballPosX) + Ki*sumErrorX + Kd*((prevBallPosX-ballPosX)) # <- não entendo o 0.0333...
    Iy = Kp*(centerY-ballPosY) + Ki*sumErrorY + Kd*((prevBallPosY-ballPosY))
        
	#Arredondar erros para 4 casas decimais
    Ix = round(Ix/10000, 4)
    Iy = round(Iy/10000, 4)
    
	#Se bola está no centro da bandeja, mete bandeja na horizontal:
    if Ix == 0 and Iy == 0:
        alpha = 0
        beta = 0
    
	# Os próximos 4 "elif"s são diferentes casos de erros nas posições da bola (Ix, Iy) e os respetivos ângulos para os servos. 
	# Neste exemplo existiam 3 servos em triângulo. Como no nosso só temos 2, as contas serão diferentes. 
	# Temos que averiguar os casos para o nosso sistema, para diferentes erros Ix e Iy.
    elif Ix != 0 and math.sqrt(Ix**2 + Iy**2) < 1:
        beta = math.atan(Iy/Ix)
        alpha = math.sqrt(Ix**2 + Iy**2) #math.asin(math.sqrt(Ix**2 + Iy**2))
        beta = math.degrees(beta)
        alpha = math.degrees(alpha)
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and math.sqrt(Ix**2 + Iy**2) < 1:
        if Iy > 0:
            beta = 90
            alpha = math.sqrt(Ix**2 + Iy**2) #math.asin(math.sqrt(Ix**2 + Iy**2))
        elif Iy < 0:
            beta = 270
            alpha = math.asin(math.sqrt(Ix**2 + Iy**2))
        alpha = math.degrees(alpha)

    elif Ix != 0 and math.sqrt(Ix**2 + Iy**2) > 1:
        beta = math.degrees(math.atan(Iy/Ix))
        alpha = 35
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and math.sqrt(Ix**2 + Iy**2) > 1:
        alpha = 35
        if Iy > 0:
            beta = 90
        elif Iy < 0:
            beta = 270
    
	#Alguma limitação de alpha:
    if alpha >= 35:
        alpha = 0
	
	#Adicionar alguma influência dos ângulos anteriores segundo o parâmetro omega. 
	#Neste caso, o novo alpha e cálculado como 20% de prevAlpha e 80% de alpha. 
	#O mesmo é realizado para beta.
    alpha = prevAlpha * omega + (1-omega) * alpha
    beta = prevBeta * omega + (1-omega) * beta
    
    prevAlpha = alpha
    prevBeta = beta
	
	#Permite arredondamento com precisão de 0.2 (honestamente não percebo o motivo disto)
    #alpha = round(round(alpha / 0.2) * 0.2, -int(math.floor(math.log10(0.2))))
    #beta = round(round(beta / 0.2) * 0.2, -int(math.floor(math.log10(0.2))))
    alpha = round(alpha, 2)
    beta = round(beta, 2)
    

	#Com alpha e beta determinados, é preciso enviar os ângulos para os servos.
	#O if serve para limitar os ângulos aos limites físicos dos servos, talvez possam ser alterados 
    #consoante os modelos no nosso hardware.
    """
    if alpha <= 35 and beta <= 360:
        # !!!!!!!! inserir código para enviar os ângulos para os servos !!!!!!!!! #
        pass
    """
	#Prints...
    #print(alpha, beta)
    #print(Ix,Iy,alpha,beta,ballPosX,ballPosY,prevBallPosX,prevBallPosY,sumErrorX,sumErrorY)

	#Cálculo de erros, que são utilizados no PID:
    if round((centerX - ballPosX), 0) == 0:
        sumErrorX = 0
    else:
        sumErrorX += (centerX - ballPosX)
    if round((centerY - ballPosY), 0) == 0:
        sumErrorY = 0
    else:
        sumErrorY += (centerY - ballPosY)
    """
    if abs(centerX - ballPosX) < abs(centerX - prevBallPosX):
        sumErrorX += (centerX - ballPosX)
    else:
        sumErrorX = 0
    if abs(centerY - ballPosY) < abs(centerY - prevBallPosY):
        sumErrorY += (centerY - ballPosY)
    else:
        sumErrorY = 0
    """
    print(sumErrorX,sumErrorY)
    return alpha, beta
#----------END OF PID CONTROL------------#
#----------------------------------------#

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
            prevBallPosX = ballPosX
            prevBallPosY = ballPosY
        img, result, ball_exists, ballPosX, ballPosY  = find_ball(img, x_cent, y_cent)
        if not prevBallPosX == 0:
            prevBallPosX = ballPosX
            prevBallPosY = ballPosY
        # Call PID function
        if ball_exists:
            alpha, beta = PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, x_cent, y_cent)
            pwm1, pwm2 = servos(alpha, beta)
            if pwm1 != 4.4 or pwm2 != 6.25:
                servo1.ChangeDutyCycle(pwm1)
                servo2.ChangeDutyCycle(pwm2)
        print(pwm1, pwm2)
        # Place green circle in center of original image
        cv2.circle(original, (400, 300), 10, (0, 255, 0), -1)
        windows(original, img, result)        
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
        # Print Frames per Second
        if time.time() - t>1:
            print("FPS: ", "%.2f" % (1.0 / (time.time() - start_time)))
            t = time.time()
    # Exit closing cv2 windows
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
