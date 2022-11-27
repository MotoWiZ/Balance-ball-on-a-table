# -*- coding: utf-8 -*-
"""
Created on Sat Nov 26 2022

@author: MotoWiZ
"""

#%%
import cv2
import time
import numpy as np
from picamera2 import Picamera2 

lower_white = (0, 0, 0)
upper_white = (0, 0, 255)
t = time.time()

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
    if np.sum(bw) > 1000:
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
    return img, result, ball_exists

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
omega = 0.2
centerX, centerY = 240,240 # <- CENTRO DA BANDEJA, não sei os valores. Não precisa de ser declarado aqui, deve 
#ser determinado noutro sítio qualquer e deve ser colocado nas variáveis globais centerX e centerY 

def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, centerX, centerY):
    global omega
    global sumErrorX, sumErrorY
    global alpha, beta, prevAlpha, prevBeta
    
	#Definição dos parâmetros:
    Kp = 1 #valor de Kp
    Ki = 1 #valor de Ki
    Kd = 1 #valor de Kd
	
	#Cálculo dos erros em X e Y:
    Ix = Kp*(centerX-ballPosX) + Ki*sumErrorX + Kd*((prevBallPosX-ballPosX)/0.0333) # <- não entendo o 0.0333...
    Iy = Kp*(centerY-ballPosY) + Ki*sumErrorY + Kd*((prevBallPosY-ballPosY)/0.0333)
    
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
    elif Ix != 0 and sqrt(Ix**2 + Iy**2) < 1:
        beta = atan(Iy/Ix)
        alpha = asin(sqrt(Ix**2 + Iy**2))
        beta = degrees(beta)
        alpha = degrees(alpha)
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) < 1:
        if Iy > 0:
            beta = 90
            alpha = asin(sqrt(Ix**2 + Iy**2))
        elif Iy < 0:
            beta = 270
            alpha = asin(sqrt(Ix**2 + Iy**2))
        alpha = degrees(alpha)

    elif Ix != 0 and sqrt(Ix**2 + Iy**2) > 1:
        beta = degrees(atan(Iy/Ix))
        alpha = 35
        if Ix < 0 and Iy >= 0:
            beta = abs(beta)
        elif Ix > 0 and Iy >= 0:
            beta = 180-abs(beta)
        elif Ix > 0 and Iy <= 0:
            beta = 180+abs(beta)
        elif Ix < 0 and Iy <= 0:
            beta = 360-abs(beta)

    elif Ix == 0 and sqrt(Ix**2 + Iy**2) > 1:
        alpha = 35
        if Iy > 0:
            beta = 90
        elif Iy < 0:
            beta = 270
	
	#Alguma limitação de alpha:
    if alpha > 35:
        alpha = 35
	
	#Adicionar alguma influência dos ângulos anteriores segundo o parâmetro omega. 
	#Neste caso, o novo alpha e cálculado como 20% de prevAlpha e 80% de alpha. 
	#O mesmo é realizado para beta.
    alpha = prevAlpha * omega + (1-omega) * alpha
    beta = prevBeta * omega + (1-omega) * beta
	
	#Permite arredondamento com precisão de 0.2 (honestamente não percebo o motivo disto)
    alpha = round(round(alpha / 0.2) * 0.2, -int(floor(log10(0.2))))
    beta = round(round(beta / 0.2) * 0.2, -int(floor(log10(0.2))))

	#Com alpha e beta determinados, é preciso enviar os ângulos para os servos.
	#O if serve para limitar os ângulos aos limites físicos dos servos, talvez possam ser alterados 
    #consoante os modelos no nosso hardware.
    if alpha <= 35 and beta <= 360:
        # !!!!!!!! inserir código para enviar os ângulos para os servos !!!!!!!!! #
	
	#Prints...
    #print(alpha, beta)
    print(Ix,Iy,alpha,beta,ballPosX,ballPosY,prevBallPosX,prevBallPosY,sumErrorX,sumErrorY)

	#Cálculo de erros, que são utilizados no PID:
    sumErrorX += (centerX-ballPosX)
    sumErrorY += (centerY-ballPosY)
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
        img, result, ball_exists = find_ball(img, x_cent, y_cent)
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
    cv2.destroyAllWindows()
