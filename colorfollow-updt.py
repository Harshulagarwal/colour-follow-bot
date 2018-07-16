import cv2
import numpy as np
import time
cap=cv2.VideoCapture(1)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

while cap.isOpened():
  ret,frame=cap.read()
  s=0
  s1=0
  s2=0
  i=240
  j=590
  j2=0
  i2=240
  if ret==True:
    #cv2.imshow("Frame",frame)
    frame=cv2.GaussianBlur(frame,(3,3),9)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    upper_red = np.array([90,60,200])
    lower_red = np.array([3,10,100])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(frame, lower_red, upper_red)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,(3,3))
    # Bitwise-AND mask and original image
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (11,11))

    frame = cv2.bitwise_and(frame,frame, mask= mask)
    cv2.imshow("res",frame)
    j1=270
    i1=190
    s=0
    while i1<290:
        while j1<370:
            px=frame[i1,j1]
            if (px[2]>px[1]) and (px[0]<px[2]) and px[2]-px[1]>15 and px[2]-px[0]>15 :

                s=s+1

                #time.sleep(.5)

            j1=j1+1
        i1=i1+1
    if s>=100 :
        print "forward"

    #print s

    #elif s<120:
    while i<290:
        while j<639:
            px1=frame[i,j]


            if (px1[2]>px1[1]) and (px1[0]<px1[2]) and px1[2]-px1[1]>15 and px1[2]-px1[0]>15 :
                s1=s1+1
            j=j+1
        i=i+1

            #print px1
            #print i
            #if (px)
            #time.sleep(.5)


    #print s1
    if s1>=40:
        print "Left"



    while i2<290:
        while j2<50:
            px1=frame[i2,j2]


            if (px1[2]>px1[1]) and (px1[0]<px1[2]) and px1[2]-px1[1]>15 and px1[2]-px1[0]>15 :
                s2=s2+1
            j2=j2+1
        i2=i2+1

            #print px1
            #print i
            #if (px)
            #time.sleep(.5)


    #print s2
    if s2>=35:
        print "Right"

      #time.sleep(.5)
    '''while j<400:
      px2=frame[j,10]
      #print
      #print j
      j=j+1
    #time.sleep(.5)'''

    '''
    if (px[2]>px[1]) and (px[0]<px[2]) and px[2]-px[1]>15 and px[2]-px[0]>15 :
        print "forward"
        s=1
        print px[2]

        #time.sleep(.5)
        i=i+1

    if (px1[2]>px1[1]) and (px1[0]<px1[2]) and px1[2]-px1[1]>15 and px1[2]-px1[0]>15 and s==0  :
        s1=1
        print "left"
            #time.sleep(.5)
        print px1[2]'''


    '''if (px2[2]>px2[1]) and (px2[0]<px2[2]) and px2[2]-px2[1]>15 and px2[2]-px2[0]>15 and s==0 and s1==0:
        print "right"
        s2=1
        #print px2[2]'''

        #time.sleep(.5)
    if s==0 and s1==0 and s2==0:
        print "stop"
    #time.sleep(.5)
    if  cv2.waitKey(1) & 0xFF == 27:
      break
    #time.sleep(1)
    # Break the loop
  else:
    break

  # When everything done, release the video capture and video write objects
cap.release()
  # Closes all the frames
