import cv2
import time
from picamera2 import Picamera2 

lower_white = (0, 0, 0)
upper_white = (0, 0, 255)
t = time.time()

# Configuration of Raspberry Pi camera (Legacy disabled in raspi-config)
def camera_config():
    picam2 = Picamera2() 
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (800, 600)}))
    picam2.start()
    return picam2

# Configure ROI with double mouse clicks
def mark_roi(event,x,y,flags,param):
  global mouseX, mouseY, nr_click
  if event == cv2.EVENT_LBUTTONDBLCLK:
      cv2.circle(img,(x,y),10,(0, 0, 255),-1)
      mouseX, mouseY = x, y
      nr_click += 1

# Crop and convert image from camera    
def get_image(picam2):
    img = picam2.capture_array()
    img = cv2.cvtColor(img,cv2.COLOR_RGBA2RGB)
    
    return img

# Find white ball on table
def find_ball(img, x_cent, y_cent):
    cX, cY = 0, 0
    hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
    # Mask image for whites
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    # Get bitwise image with mask    
    result = cv2.bitwise_and(img, img, mask = mask)
    
    # Find center of white ball   
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    
    # Place red circle and text in center of white ball
    cv2.circle(result, (cX, cY), 10, (0, 0, 255), -1)
    cv2.putText(result, "Object center", (cX - 110, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    # Place green circle in center of result
    cv2.circle(result, (x_cent, y_cent), 10, (0, 255, 0), -1)
    # Place green circle in center of reduced image
    cv2.circle(img, (x_cent, y_cent), 10, (0, 255, 0), -1)
    
  
    return img, result

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
    cv2.moveWindow("Reduced image", 420,100)
    
    # Masked image window
    cv2.namedWindow("Masked image", cv2.WINDOW_NORMAL)
    cv2.imshow("Masked image",result)
    cv2.resizeWindow("Masked image", 400, 300) 
    cv2.moveWindow("Masked image", 840,100) 

if __name__ == "__main__":
    picam2 = camera_config()
    img = get_image(picam2)
    cv2.namedWindow("Pick points")
    cv2.moveWindow("Pick points", 320,100) 
    cv2.setMouseCallback("Pick points",mark_roi)
    nr_click = 0
    prev_nr_click = nr_click
    x_pos, y_pos = [], []
    
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
    
    while True:
        # Start timer to count FPS
        start_time = time.time()
        
        original = get_image(picam2)
        img = original.copy()
        
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
        
        img, result = find_ball(img, x_cent, y_cent)
        
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
