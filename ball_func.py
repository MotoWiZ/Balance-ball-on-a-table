import cv2
import time
from picamera2 import Picamera2 

# Define CV2 Windows
cv2.namedWindow("Original image", cv2.WINDOW_NORMAL)
cv2.namedWindow("Masked image", cv2.WINDOW_NORMAL)

cX, cY = 0, 0
lower_white = (0, 0, 0)
upper_white = (0, 0, 255)
t = time.time()

# Configuration of Raspberry Pi camera (Legacy disabled in raspi-config)
def camera_config():
    picam2 = Picamera2() 
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800, 600)}))
    picam2.start()
    return picam2

# Crop and convert image from camera    
def get_image(picam2):
    img = picam2.capture_array()
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    img = cv2.cvtColor(img,cv2.COLOR_RGBA2RGB)
    
    return img, hsv

# Find white ball on table
def find_ball(img):
    # Mask image for whites
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Get bitwise image with mask    
    result = cv2.bitwise_and(img, img, mask = mask)
    
    # Find center of white ball   
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    
    # Place red circle in center of white ball
    cv2.circle(result, (cX, cY), 10, (0, 0, 255), -1)
    # Place green circle and text in center of result
    cv2.circle(result, (400, 300), 10, (0, 255, 0), -1)
    cv2.putText(result, "Object center", (cX - 110, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    # Place green circle in center of original image
    cv2.circle(img, (400, 300), 10, (0, 255, 0), -1)
    
    return img, result

def windows(img, result):
    # Original image window
    cv2.imshow("Original image",img)
    cv2.resizeWindow("Original image", 400, 300)
    cv2.moveWindow("Original image", 10,100)
    
    # Masked image window
    cv2.imshow("Masked image",result)
    cv2.resizeWindow("Masked image", 400, 300) 
    cv2.moveWindow("Masked image", 440,100) 

if __name__ == "__main__":
    picam2 = camera_config()
    
    while True:
        # Start timer to count FPS
        start_time = time.time()
        
        img, hsv = get_image(picam2)
        
        img, result = find_ball(img)
        
        windows(img, result)        

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break
    
        # Print Frames per Second
        if time.time() - t>1:
            print("FPS: ", "%.2f" % (1.0 / (time.time() - start_time)))
            t = time.time()
        
    # Exit closing cv2 windows
    cv2.destroyAllWindows()
