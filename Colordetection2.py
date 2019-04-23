#python color_tracking.py --video balls.mp4
#python color_tracking.py

# import the necessary packages
#from collections import deque
import numpy as np
import argparse
import imutils
import cv2
import pyfirmata
import math  
import time 
#import urllib #for reading image from URL


# construct the argument parse and parse the arguments
#ap = argparse.ArgumentParser()
#ap.add_argument("-v", "--video",
 #   help="path to the (optional) video file")
#ap.add_argument("-b", "--buffer", type=int, default=64,
 #   help="max buffer size")
#args = vars(ap.parse_args())
 
# define the lower and upper boundaries of the colors in the HSV color space
lower = {'green':(66, 122, 129), 'blue':(97, 100, 117), 'yellow':(23, 59, 119)} #assign new item lower['blue'] = (93, 10, 0)
upper = {'green':(86,255,255), 'blue':(117,255,255), 'yellow':(54,255,255)}

# define standard colors for circle around the object
colors = {'green':(0,255,0), 'blue':(255,0,0), 'yellow':(0, 255, 217)}
AngleCam = 0 
Cy = 0 
Cx = 0 
Cye = 0 
error = 0 
Elbowdynamic = 0 
Elbowdynamic2 = 0 
Shoulderdynamic = 0 
Anglebase = 0 
Xreal = 0 
Yreal = 0 
#pts = deque(maxlen=args["buffer"])
hardware = pyfirmata.ArduinoMega("COM12") # hardware connection 
base = hardware.get_pin('d:5:s')
shoulder = hardware.get_pin('d:6:s')
shoulderl = hardware.get_pin('d:10:s')
elbow = hardware.get_pin('d:7:s')
wrist = hardware.get_pin('d:4:s')
wristrotate = hardware.get_pin('d:8:s')
gripper = hardware.get_pin('d:9:s')
gripper.write(0)
# if a video path was not supplied, grab the reference
# to the webcam
#cv2.namedWindow("Window")
camera = cv2.VideoCapture(1)
# otherwise, grab a reference to the video file
# keep looping
def Settingfreeze(sholAngle,elboAngle,wristAngle,WristrotAngle,GripperAngle):
     shoulder.write(sholAngle)
     elbow.write(elboAngle)
     wrist.write(wristAngle)
     wristrotate.write(WristrotAngle)
    # gripper.write(GripperAngle)

  #   elif key == 'blue': 
   #           shoulder.write(sholAngle-10)
    #          elbow.write(elboAngle-10)
     #         wrist.write(wristAngle+20)
      #        wristrotate.write(WristrotAngle)
       #3       gripper.write(GripperAngle-110)
def AngleElbow(l1,l2,Cy):  
       Thetaelbow = math.acos((l1*l1 + l2*l2 - Cy*Cy)/(2*l1*l2))
       return Thetaelbow
def baseview(x,y):
     Theta = math.degrees(math.atan(y/x))
     return Theta 
def distance(x,y): 
    d = math.sqrt(x*x + y*y)
    return d 
while True:
    # grab the current frame
    (grabbed, frame) = camera.read()
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
     

    #IP webcam image stream 
    #URL = 'http://10.254.254.102:8080/shot.jpg'
    #urllib.urlretrieve(URL, 'shot1.jpg')
    #frame = cv2.imread('shot1.jpg')

 
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=640,height=480)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    #for each color in dictionary check object in frame
    for key, value in upper.items():
        # construct a mask for the color from dictionary`1, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        kernel = np.ones((9,9),np.uint8)
        mask = cv2.inRange(hsv, lower[key], upper[key])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
            # only proceed if the radius meets a minimum size. Correct this value for your obect's size
            if radius > 0.5:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 2)
                print(x,y,key)       
               # base.write(180-(x/640)*180) # Base view
                base.write(65+baseview(x,y))
              #  if Anglebase < 90: 
               #          base.write(180-baseview(x,y))          
                 # Y distance output for calculation 
                Cy = (y/480)*247.7 + 2*radius 
                Cx = (x/640)*722.2 + 2*radius
                error = 3*math.pow(2,(Cy/60))
                Cye = Cy + 2*error
                #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                Xreal = (x/630.5)*481  
                Yreal = (y/147.5)*165.8
                print("Only y value")
                print(y,Yreal)
                print("Only x value")
                print(x,Xreal)
                #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                print("Real distance")
                print(math.sqrt(Xreal*Xreal + Yreal*Yreal))
                #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                print("Y-distance translation")
                print(Cy) 
                print(Cye)
                print("Pixel distance")
                print(math.sqrt(x*x + y*y))
                Elbowdynamic = ((380 - Cye)/248)*80 + 20
                Elbowdynamic2 = ((380- Cye)/248)*80 
                print(Elbowdynamic)
                print(Elbowdynamic2)
                Shoulderdynamic = (180 - Elbowdynamic2)/2
                print(Shoulderdynamic)
                elbow.write(Elbowdynamic)
                shoulder.write(180 - Shoulderdynamic)
                shoulderl.write(180 - Shoulderdynamic)
                wrist.write(0)
                #Anglebase = 180-(x/640)*180
                Anglebase = 65 + baseview(x,y)
                print("Angle Base joint:")
                print(60+baseview(x,y))
                #print(180-(x/640)*180)
                AngleCam = 180-(x/640)*180 
                if key == "yellow":
                     #print("Yellow detected")
                     wrist.write(0)
                     gripper.write(0)
                     shoulder.write(180-Shoulderdynamic + 5)
                     elbow.write(Elbowdynamic +5)
                     if Anglebase >=90  or Anglebase < 180:
                        wrist.write(5)
                        gripper.write(0)
                        time.sleep(0.5)
                        gripper.write(50)
                       # base.write(10)
                        #gripper.write(0)
                        time.sleep(0.8)
                if key == "blue":
                     wrist.write(0)
                     gripper.write(0)
                     shoulder.write(180-Shoulderdynamic + 5)
                     elbow.write(Elbowdynamic +5)
                     if Anglebase >=90  or Anglebase < 180:
                        wrist.write(5)
                        gripper.write(0)
                        time.sleep(0.5)
                        gripper.write(50)
                       # base.write(10)
                        #gripper.write(0)
                        time.sleep(0.8)

                else: 
                    gripper.write(0)
                    wrist.write(20)
                #GripperWorks(key,120,120,20,0,50)
               

        
              #  elif(key != "blue"):
               #     if key != "green":
                #       base.write(75+math.degrees(Cameraviewbase(x,y))) # Base view
                 #      Settingfreeze(80,120,30,0,50)
         
                cv2.putText(frame,key + "cube", (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)

     
    # show the frame to our screen
    cv2.imshow("Frame", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
