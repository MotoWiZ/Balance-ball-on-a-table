import cv2

def mark_roi(event,x,y,flags,param):
  global mouseX, mouseY, nr_click
  if event == cv2.EVENT_LBUTTONDBLCLK:
      cv2.circle(img,(x,y),10,(0, 0, 255),-1)
      mouseX,mouseY = x,y
      nr_click += 1

        
img = cv2.imread('test.jpg')
cv2.namedWindow('image')
cv2.setMouseCallback('image',mark_roi)
nr_click = 0
prev_nr_click = nr_click

while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if nr_click == 3:
      break
    elif nr_click != prev_nr_click:
      prev_nr_click = nr_click
      print (mouseX,mouseY,nr_click)
