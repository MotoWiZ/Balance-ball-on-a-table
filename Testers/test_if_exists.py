import cv2
import numpy as np

def verify(img):
  hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)

  mask = cv2.inRange(hsv, (0, 0, 0), (0, 0, 255))

  result = cv2.bitwise_and(img, img, mask = mask)

  gray = cv2.cvtColor(result,cv2.COLOR_RGBA2GRAY)
  
  (thresh, im_bw) = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
  n = np.sum(im_bw)
  
  return (im_bw, n)

img1 = cv2.imread('test1.jpg')
img2 = cv2.imread('test2.jpg')

cv2.namedWindow('image1')
cv2.namedWindow('image2')

while True:
  im_bw, n = verify(img1)
  if n > 1000:
    cv2.putText(im_bw, "Ball detected", (30, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
  else:
    cv2.putText(im_bw, "No ball detected", (30, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
  cv2.imshow('image1',im_bw)
  
  im_bw, n = verify(img2)
  if n > 100:
    cv2.putText(im_bw, "Ball detected", (30, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
  else:
    cv2.putText(im_bw, "No ball detected", (30, 100),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
  cv2.imshow('image2',im_bw)
  
  # Press 'q' to quit
  if cv2.waitKey(1) & 0xFF == ord('q'): 
    break
# Exit closing cv2 windows
cv2.destroyAllWindows()