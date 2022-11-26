import cv2
import time
from picamera2 import Picamera2 

picam2 = Picamera2() 
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800, 600)}))
picam2.start() 

cv2.namedWindow("Original image", cv2.WINDOW_NORMAL)
cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)

cX, cY = 0, 0

lower_white = (0, 0, 0)
upper_white = (0, 0, 255)


while True:
    start_time = time.time()
    img = picam2.capture_array()
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    img = cv2.cvtColor(img,cv2.COLOR_RGBA2RGB)
    
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    result = cv2.bitwise_and(img, img, mask = mask)
        
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    
    cv2.circle(result, (cX, cY), 10, (0, 0, 255), -1)
    cv2.circle(result, (400, 300), 10, (0, 255, 0), -1)
    cv2.circle(img, (400, 300), 10, (0, 255, 0), -1)
    cv2.putText(result, "Centro do objeto", (cX - 110, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    cv2.imshow("Original image",img)
    cv2.resizeWindow("Original image", 400, 300)
    cv2.moveWindow("Original image", 10,100)
    
    cv2.imshow("Mask",result)
    cv2.resizeWindow("Mask", 400, 300) 
    cv2.moveWindow("Mask", 440,100) 
    
  
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
    print("FPS: ", 1.0 / (time.time() - start_time))
cv2.destroyAllWindows()
