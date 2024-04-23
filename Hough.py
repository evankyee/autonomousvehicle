# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
null.tpl [markdown]
# # Demo Lines Detection with Hough Transform
# 

# %%
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

cap = cv2.VideoCapture('vid.mp4')

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur1 = cv2.blur(gray,(5,5))
        _, th_img = cv2.threshold(blur1,200,255,cv2.THRESH_BINARY)
        blur = cv2.blur(th_img,(5,5))

        edge = cv2.Canny(blur, 180, 255, 30)
        lines = cv2.HoughLines(edge,1,np.pi/180, 200,20,0)

        if lines is not None:
            for i in range(len(lines)):
                rho, theta = lines[i][0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho; y0 = b*rho
                
                xs = [int(x0 + 1000*(-b)), int(x0 - 1000*(-b))]
                ys = [int(y0 + 1000*(a)), int(y0 - 1000*(a))]
                cv2.line(frame,(int(x0 + 1000*(-b)), int(y0 + 1000*(a))),( int(x0 - 1000*(-b)),int(y0 - 1000*(a)) ), (255, 0, 0), 2)


        #cv2.imshow('frame',gray)
        cv2.imshow('thresholded',th_img)
        cv2.imshow('edge',edge)
        
        cv2.imshow('lines',frame)
        time.sleep(0.005)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


# %%
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

cap = cv2.VideoCapture('vid.mp4')

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur1 = cv2.blur(gray,(5,5))
        _, th_img = cv2.threshold(blur1,200,255,cv2.THRESH_BINARY)
        blur = cv2.blur(th_img,(5,5))

        edge = cv2.Canny(blur, 180, 255, 30)

       
        # LSD
        lsd = cv2.createLineSegmentDetector(0)
        #Detect lines in the image
        lines = lsd.detect(edge)[0] #Position 0 of the returned tuple are the detected lines
        #Draw detected lines in the image
        drawn_img = lsd.drawSegments(frame,lines)

        #cv2.imshow('frame',gray)
        cv2.imshow('thresholded',th_img)
        cv2.imshow('edge',edge)
        
        cv2.imshow('lines',frame)
        time.sleep(0.005)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


# %%



