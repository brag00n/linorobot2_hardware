# This is simple script is use to get images
# from the Android App IP Webcam
# Developer: Michael Jalloh
############################################
# Special thanks to Peter Lunk on hackster.io

import cv2
from urllib.request import urlopen
import numpy as np
import pygame
import re
import random
import math
import time
import operator
#from Bamboo4W.interaction.FaceRecognition import FaceRecognition
from array import array
#from lib2to3.pgen2.tokenize import Ignore
import urllib.request
#from django.template.defaultfilters import length
#from numpy import integer
import imutils

isInit=False

def initDisplaymask(dummy=None):
    global isInit
    print("isInit="+str(isInit))
    if (not isInit):
        isInit=True
        cv2.setMouseCallback('Applied Mask', onmouse)
        cv2.createTrackbar('H min', 'Applied Mask', 1, 256, displayMask)
        cv2.createTrackbar('V min', 'Applied Mask', 1, 256, displayMask)
        cv2.createTrackbar('C min', 'Applied Mask', 1, 256, displayMask)
        cv2.createTrackbar('H max', 'Applied Mask', 1, 256, displayMask)
        cv2.createTrackbar('V max', 'Applied Mask', 1, 256, displayMask)
        cv2.createTrackbar('C max', 'Applied Mask', 1, 256, displayMask)
    
def displayMask(mask=None,greenLower = None,greenUpper = None):
    
    if (greenLower is not None):
        (hMin,vMin,cMin)=greenLower
        cv2.setTrackbarPos('H min', 'Applied Mask',hMin)
        cv2.setTrackbarPos('V min', 'Applied Mask',vMin)
        cv2.setTrackbarPos('C min', 'Applied Mask',cMin)

    if (greenUpper is not None):
        (hMax,vMax,cMax)=greenUpper
        cv2.setTrackbarPos('H max', 'Applied Mask',hMax)
        cv2.setTrackbarPos('V max', 'Applied Mask',vMax)
        cv2.setTrackbarPos('C max', 'Applied Mask',cMax)
    
    if (isInit):
        hMin= max(cv2.getTrackbarPos('H min', 'Applied Mask'),0)
        vMin= max(cv2.getTrackbarPos('V min', 'Applied Mask'),0)
        cMin= max(cv2.getTrackbarPos('C min', 'Applied Mask'),0)
        greenLower=(hMin,vMin,cMin)
        
        vMax= max(cv2.getTrackbarPos('V max', 'Applied Mask'),0)
        hMax= max(cv2.getTrackbarPos('H max', 'Applied Mask'),0)
        cMax= max(cv2.getTrackbarPos('C max', 'Applied Mask'),0)
        greenUpper=(hMax,vMax,cMax)
    
    cv2.imshow('Applied Mask',mask)
    #return (29, 86, 6),(64, 255, 255)
    return greenLower,greenUpper


def onmouse(event, x, y, flags, param):
    global seed_pt
    if flags & cv2.EVENT_FLAG_LBUTTON:
        seed_pt = x, y
        print('seed_pt='+str(x)+','+str(y))
        displayMask()
    
MIN_MATCH_COUNT = 4
class RobotObject():
    def __init__(self):
        self.objects=array
        self.TrackedObject=None
        self.TrackedObjectLife=0
        self.trackedId=-1
        self.TrackedObjectName="unknow"
        self.image=None
        self.SearchFrequency=20

    def _searchObject(self,pImg,pOrigin):

        #area,pImg
        return None,pImg

    def _displayTrackingInfo(self,pImg,pOrigin):
        #Display traking informations
                #Draw centered marker
        cv2.rectangle(pImg,(int(320)-50,int(240)-50),(int(320)+50,int(240)+50),(0,255,0),2)
        cv2.drawMarker(pImg, (int(320),int(240)), (0,255,0),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)      

        if (self.getTrackedObject() is not None):
            
            #Record unknow faces
            (ax,ay,aw,ah)=self.getTrackedObject()

            #Display traking informations
            cv2.drawMarker(pImg, (int(ax+(aw/2)),int(ay+(ah/2))), (0,0,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)
            cv2.putText(pImg, (self.TrackedObjectName+' ('+str(self.trackedId)+')'), (ax-25,ay-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

        return self
    
    def _displayTrackedObject(self,pImg,pOrigin):
        if (self.getTrackedObject() is not None):
            (x,y,w,h)=self.getTrackedObject()
            cv2.rectangle(pImg,(x,y),(x+w,y+h),(0,0,255),2)
        return self
    
    def getImage(self):
        return self.image
    
    def getTrackedObject(self): 
        return self.TrackedObject

    def getObjectsInImage(self,pImg,pImgOrigin):
        
        #Check parameters
        if (pImg is None or pImgOrigin is None):
            return self

        
        self._displayTrackingInfo(pImg,pImgOrigin)
        
        #Search object with low frequency
        self.TrackedObjectLife=self.TrackedObjectLife+1
        if (self.SearchFrequency==0 or self.TrackedObjectLife % self.SearchFrequency ==0):
            if (self.getTrackedObject() is None or self.SearchFrequency==0):
                self.objects,pImg = self._searchObject(pImg,pImgOrigin)
                self.setTrackedObject(self.objects)
        if self.TrackedObjectLife>=100:
            self.TrackedObjectLife=0
            
        #If objects found
        if (self.getTrackedObject() is not None):
            #Init or update Object tracking
            if (self.SearchFrequency >0):
                if self.trackedId == -1:
                    self.initTrackedObjectId(self.getTrackedObject(),pImg,pImgOrigin)
                else:
                    self.getTrackedObjectId(self.getTrackedObject(),pImg,pImgOrigin)
                
            #Display tracker
            self._displayTrackedObject(pImg,pImgOrigin)
                
        self.image=pImg
        #cv2.imshow("matched", pImg);
        return self

    def setTrackedObject(self,pObject):
        self.TrackedObject=pObject
        
    def initTrackedObjectId(self,pTrackedObject,pImage,pImgOrigin):

        if (pTrackedObject is None):
            return None
        (x,y,w,h)=pTrackedObject
        
        #Enlarge tracked area size
        pTrackedObject=(int(x-(x*0.2)/2),int(y-(y*0.2)/2),int(w+w*0.2),int(h+h*0.2))
        (x,y,w,h)=pTrackedObject
        
        #If box out of window
        if (x<0 or y<0):
            return None
        
        #Get sub area picture and convert it into HSV
        roi = pImgOrigin[y:y+h, x:x+w]
        self.hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        #Get mask in color range
        mask = cv2.inRange(self.hsv_roi, np.array((0., 30.,32.)), np.array((180.,255.,255.)))
        
        #Normalize color histogram
        self.roi_hist = cv2.calcHist([self.hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        
        #Set meanShift algo
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1)
        if (self.trackedId == -1):
            self.trackedId=random.randint(1,10000)
        return self.trackedId
    
    def getTrackedObjectId(self,pTrackedObject,pImage,pImgOrigin):
        if (pTrackedObject is None):
            return None
        hsv = cv2.cvtColor(pImgOrigin, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0,180], 1)
        #https://docs.opencv.org/3.4.0/db/df8/tutorial_py_meanshift.html
        ret, track_window = cv2.meanShift(dst, pTrackedObject, self.term_crit)
        (x,y,w,h)=track_window
        self.TrackedObject = track_window
        cv2.rectangle(pImage,(x,y),(x+w,y+h),(255,0,255),2)
        if ret > 15:
            self.TrackedObject=None
            self.trackedId=-1
            self.TrackedObjectLife=0
            self.TrackedObjectName="unknow"
        return self.trackedId
    
        
class Ground(RobotObject):
    def __init__(self):
        RobotObject.__init__(self)
        
    def detectGround(self,pImg,pImgOrigin):
        
        if (pImgOrigin is None):
            return
        h, w = pImgOrigin.shape[:2]
        print("h="+str(h)+", w="+str(w))
        #cv2.imshow('roi', pImgOrigin[280:340, 330:390])
        #cv2.imshow('roi', pImgOrigin[h-100:h-10, 10:w-10])
        roi=pImgOrigin[h-100:h-10, 10:w-10]
        cv2.rectangle(pImg,(10,h-100),(w-10,h-10),(0,255,0),2)
        hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
         
        target = pImgOrigin
        hsvt = cv2.cvtColor(target,cv2.COLOR_BGR2HSV)
         
        # calculating object histogram
        roihist = cv2.calcHist([hsv],[0, 1], None, [256, 256], [0, 180, 0, 256] )
         
        # normalize histogram and apply backprojection
        #cv2.normalize(roihist,roihist,0,255,cv2.NORM_MINMAX)
        dst = cv2.calcBackProject([hsvt],[0,1],roihist,[0,180,0,256],1)
         
        # Now convolute with circular disc
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
        cv2.filter2D(dst,-1,disc,dst)
         
        # threshold and binary AND
        ret,thresh = cv2.threshold(dst,50,255,0)
        thresh = cv2.merge((thresh,thresh,thresh))
        res = cv2.bitwise_and(target,thresh)
         
        pImg=self.CheckGround(thresh,pImg)
        
    def CheckGround(self,img,pImageOut):
    
        StepSize = 5
        EdgeArray = []
    
        imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)   #convert img to grayscale and store result in imgGray
        imgGray = cv2.bilateralFilter(imgGray,9,30,30) #blur the image slightly to remove noise             
        imgEdge = cv2.Canny(imgGray, 50, 100)             #edge detection
            # Mask used to flood filling.
        # Notice the size needs to be 2 pixels than the image.
        h, w = img.shape[:2]
        mask = np.zeros((h + 2, w + 2), np.uint8)
        mask[:] = 0
        floodImg = img.copy()
    
        lower_color = 97#np.array([1, 12, 13])
        upper_color = 52#np.array([1, 12, 13])
        flags=4
        flags |= cv2.FLOODFILL_FIXED_RANGE
        #cv2.floodFill(img,mask, (771,551),(255, 255, 255),(lower_color,)*3,(upper_color,)*3,flags)
        #th, im_th = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)
        cv2.imshow('imgEdge1', floodImg)
        
        
        imagewidth = imgEdge.shape[1] - 1
        imageheight = imgEdge.shape[0] - 1
        maxY=0
        EdgeObstacleStart=None
        EdgeObstacleEnd=None
        for j in range (0,imagewidth,StepSize):    #for the width of image array
            for i in range(imageheight-5,0,-1):    #step through every pixel in height of array from bottom to top #Ignore first couple of pixels as may trigger due to undistort
                                                 
                if imgEdge.item(i,j) == 255:       #check to see if the pixel is white which indicates an edge has been found
                    EdgeArray.append((j,i))        #if it is, add x,y coordinates to ObstacleArray
                    #print ("EdgeArray.item("+str(j)+","+str(i)+")")  
                    if (maxY>i):
                        maxY=i
                        EdgeObstacleStart=(i,j)
                        ObstacleDetected=True
                    
                    break                          #if white pixel is found, skip rest of pixels in column
                
        
        for x in range (len(EdgeArray)-1):      #draw lines between points in ObstacleArray 
            cv2.line(pImageOut, EdgeArray[x], EdgeArray[x+1],(0,0,255),5) 
        for x in range (len(EdgeArray)):        #draw lines from bottom of the screen to points in ObstacleArray
            cv2.line(pImageOut, (x*StepSize,imageheight), EdgeArray[x],(0,0,255),1)
    
        return pImageOut
                    
class Faces(RobotObject):
    def __init__(self,pfaceRecognition=None):
        RobotObject.__init__(self)
        self.trackedObjectOccurency={}
        self.faceRecognition = pfaceRecognition#FaceRecognition()
        self.SearchFrequency=40

 
    def _searchObject(self,pImg,pOrigin):
        bigestArea=None
        
        #search Face using Face classifier
        face_cascade = cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_frontalface_alt2.xml')
        areas = face_cascade.detectMultiScale(pOrigin, 1.3, 5)
        
        #IF not found Find Face using eye classifier
        if (areas is None or len(areas)==0):
            areas = self._getFacesWithEyesInImage2(pOrigin)
            
        #Keep bigest Face
        if (areas is not None):
            for (x,y,w,h) in areas:
                
                bx, by, bwidth, bheight=(x,y,w,h)
                cv2.drawMarker(pImg, (int(bx+(bwidth/2)),int(by+(bheight/2))), (255,255,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)
                if (bigestArea is None):
                    bigestArea=(x,y,w,h)
                (a,b,c,d)=bigestArea
                    
                if ((w*h) > (c*d)):
                        bigestArea=(x,y,w,h)
                   
        return bigestArea,pImg
    
    def _displayTrackedObject(self,pImg,pOrigin):
        if (self.getTrackedObject() is not None):
            (x,y,w,h)=self.getTrackedObject()
            cv2.rectangle(pImg,(x,y),(x+w,y+h),(0,0,255),2)
        return self
    
    def _displayTrackingInfo(self,pImg,pOrigin):
        
        #Display traking informations
        if (self.getTrackedObject() is not None):
 
            #Record unknow faces
            (ax,ay,aw,ah)=self.getTrackedObject()
            if (self.TrackedObjectName is not None and self.TrackedObjectName == "unknow"):
                self.recordImage(self.getTrackedObject(),pOrigin)

            #Display traking informations
            cv2.drawMarker(pImg, (int(ax+(aw/2)),int(ay+(ah/2))), (0,0,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)
            cv2.putText(pImg, (self.TrackedObjectName+' ('+str(self.trackedId)+')'), (ax-25,ay-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)

        return self
    
    def recordImage(self,area,pImage):
        if (area is not None):
            try:
                (x,y,w,h)=area
                yMargin=int( (h*(0.2)) ) #Add 20%
                xMargin=int( (w*(0.1)) ) #Add 10%
                #crop_img=self._imcrop(pImgOrigin,(max(x-xMargin,0),max(y-yMargin,0),min(x+w+xMargin,640),min(y+h+yMargin,480)))
                crop_img = pImage[max(y-yMargin,0):min(y+h+yMargin,480), max(x-xMargin,0):min(x+w+xMargin,640)]

                b,g,r = cv2.split(crop_img)
                crop_img= cv2.merge([r,g,b]) 
            except Exception as ex:
                None
    
            # compose our image back but this time as red, green and blue  
            cv2.imshow("Record1", crop_img)
            cv2.imwrite("resources/Faces/unknows/record"+str(random.randint(1,10000))+"_image.png", crop_img)
            print("Face recored")
        return self
    
    def setTrackedId(self,pTrackedId):
        self.trackedId=pTrackedId
        
    def _getFacesWithEyesInImage2(self,Image):
        if (Image is None):
            return None
        Theta = 0
        rows, cols, channels = Image.shape
        glass_cas = cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_eye_tree_eyeglasses.xml')
        face =      cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_frontalcatface.xml')
        glass = glass_cas.detectMultiScale(Image) 
        if (len(glass)==0) :
            return None                                           # This ditects the eyes
        for (sx, sy, sw, sh) in glass:
            if glass.shape[0] == 2:                                                             # The Image should have 2 eyes
                if glass[1][0] > glass[0][0]:
                    DY = ((glass[1][1] + glass[1][3] / 2) - (glass[0][1] + glass[0][3] / 2))    # Height diffrence between the glass
                    DX = ((glass[1][0] + glass[1][2] / 2) - glass[0][0] + (glass[0][2] / 2))    # Width diffrance between the glass
                else:
                    DY = (-(glass[1][1] + glass[1][3] / 2) + (glass[0][1] + glass[0][3] / 2))   # Height diffrence between the glass
                    DX = (-(glass[1][0] + glass[1][2] / 2) + glass[0][0] + (glass[0][2] / 2))   # Width diffrance between the glass
    
                if (DX != 0.0) and (DY != 0.0):                                                 # Make sure the the change happens only if there is an angle
                    Theta = math.degrees(math.atan(round(float(DY) / float(DX), 2)))            # Find the Angle
                    #print ("Theta  " + str(Theta))
    
                    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), Theta, 1)                 # Find the Rotation Matrix
                    Image = cv2.warpAffine(Image, M, (cols, rows))                                        # UNCOMMENT IF YOU WANT TO SEE THE
    
                #self.recordImage(Image)
                return face.detectMultiScale(Image, 1.3, 5)                                # This detects a face in the image

    def _getFacesWithEyesInImage(self,Image):
        Theta = 0
        if (Image is None):
            return None
        
        rows, cols, channels = Image.shape
        glass_cas = cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_eye_tree_eyeglasses.xml')
        face = cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_frontalcatface.xml')
        glass = glass_cas.detectMultiScale(Image)                                               # This ditects the eyes
        for (sx, sy, sw, sh) in glass:
            if glass.shape[0] == 2:                                                             # The Image should have 2 eyes
                if glass[1][0] > glass[0][0]:
                    DY = ((glass[1][1] + glass[1][3] / 2) - (glass[0][1] + glass[0][3] / 2))    # Height diffrence between the glass
                    DX = ((glass[1][0] + glass[1][2] / 2) - glass[0][0] + (glass[0][2] / 2))    # Width diffrance between the glass
                else:
                    DY = (-(glass[1][1] + glass[1][3] / 2) + (glass[0][1] + glass[0][3] / 2))   # Height diffrence between the glass
                    DX = (-(glass[1][0] + glass[1][2] / 2) + glass[0][0] + (glass[0][2] / 2))   # Width diffrance between the glass
    
                if (DX != 0.0) and (DY != 0.0):                                                 # Make sure the the change happens only if there is an angle
                    Theta = math.degrees(math.atan(round(float(DY) / float(DX), 2)))            # Find the Angle
                    #print ("Theta  " + str(Theta))
    
                    M = cv2.getRotationMatrix2D((cols / 2, rows / 2), Theta, 1)                 # Find the Rotation Matrix
                    Image = cv2.warpAffine(Image, M, (cols, rows))
                    # cv2.imshow('ROTATED', Image)                                              # UNCOMMENT IF YOU WANT TO SEE THE
    
                    Face2 = face.detectMultiScale(Image, 1.3, 5)                                # This detects a face in the image
                    for (FaceX, FaceY, FaceWidth, FaceHeight) in Face2:
                        FaceHeight=int(round(FaceHeight*1.2))
                        CroppedFace = Image[FaceY: FaceY + FaceHeight, FaceX: FaceX + FaceWidth]
                        #b,g,r = cv2.split(CroppedFace)

                        # compose our image back but this time as red, green and blue
                        #CroppedFace= cv2.merge([r,g,b])
                        # cv2.imshow('ROTATED', CroppedFace) 
                        return CroppedFace
     
                               
    def getObjectsInImage2(self,pImg,pImgOrigin):
        
        if (pImg is None):
            self.image=None
            return self
        gray = cv2.cvtColor(pImg, cv2.COLOR_BGR2GRAY)

        #Draw centered marker
        cv2.rectangle(pImg,(int(320)-50,int(240)-50),(int(320)+50,int(240)+50),(0,255,0),2)
        cv2.drawMarker(pImg, (int(320),int(240)), (0,255,0),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)      
        self.TrackedObjectLife=self.TrackedObjectLife+1
        
        if self.TrackedObjectLife>=100:
            self.TrackedObjectLife=0
                
        if self.getTrackedObject() is None or self.TrackedObjectLife%20==0:
            self.TrackedObjectLife=self.TrackedObjectLife+1
            face_cascade = cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_frontalface_alt2.xml')
            self.objects = face_cascade.detectMultiScale(pImgOrigin, 1.3, 5)
            if (self.objects is None or len(self.objects)==0):
                self.objects = self._getFacesWithEyesInImage2(pImgOrigin)
            if (self.objects is not None):
                for (x,y,w,h) in self.objects:
                    #self.recordImage(pImgOrigin)
                    self.setTrackedObject((x,y,w,h))
                    #cv2.rectangle(pImg,(x,y),(x+w,y+h),(255,0,0),2)

        if self.getTrackedObject() is not None:
            if self.trackedId == -1:
                self.initTrackedObjectId(self.getTrackedObject(),pImg,pImgOrigin)
            else:
                self.getTrackedObjectId(self.getTrackedObject(),pImg,pImgOrigin)

        #Display traking informations
        if (self.getTrackedObject() is not None):
            (ax,ay,aw,ah)=self.getTrackedObject()
            if (self.TrackedObjectName is not None and self.TrackedObjectName == "unknow"):
                self.recordImage(pImgOrigin)
            #cv2.(pImg,(int(ax),int(ax+aw)),(int(ay),int(ay+ah)),(0,0,255),2)
            cv2.drawMarker(pImg, (int(ax+(aw/2)),int(ay+(ah/2))), (0,0,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)
            cv2.putText(pImg, (self.TrackedObjectName+' ('+str(self.trackedId)+')'), (ax-25,ay-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
            
        
        # split our image color_space into blue, green, red components
        #b,g,r = cv2.split(pImg)

        # compose our image back but this time as red, green and blue
        #self.image= cv2.merge([r,g,b])
        return self
    
    def setTrackedObject2(self,pTrackedObject):
        if self.TrackedObject is None: 
            self.TrackedObject=pTrackedObject
        else:
            #Select the biggest face
            (a,b,c,d)=self.TrackedObject
            (x,y,w,h)=pTrackedObject
            self.TrackedObjectLife=self.TrackedObjectLife+1
            if ((w*h) > (c*d)):
                self.TrackedObject=pTrackedObject
    
    def initTrackedObjectId(self,pTrackedObject,pImage,pImgOrigin):
        # Create mask and normalized histogram
        (c,r,w,h)=pTrackedObject
        roi = pImage[r:r+h, c:c+w]
        self.hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv_roi, np.array((0., 30.,32.)), np.array((180.,255.,255.)))
        self.roi_hist = cv2.calcHist([self.hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1)
        self.trackedId=self.setFaceReconnitionTrackedObjectId(pTrackedObject,pImage,pImgOrigin)
        if (self.trackedId == -1):
            self.trackedId=random.randint(1,10000)
            print("ID reseted")
        return self.trackedId
    
    def getTrackedObjectId(self,pTrackedObject,pImage,pImgOrigin):
        hsv = cv2.cvtColor(pImage, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0,180], 1)
        #https://docs.opencv.org/3.4.0/db/df8/tutorial_py_meanshift.html
        ret, track_window = cv2.meanShift(dst, pTrackedObject, self.term_crit)
        self.TrackedObject = track_window
        #(x,y,w,h)=track_window
        #cv2.rectangle(pImage,(x,y),(x+w,y+h),(0,0,255),2)
        self.trackedId=self.setFaceReconnitionTrackedObjectId(pTrackedObject,pImage,pImgOrigin)
        if ret > 12:
            print("Board Ret="+str(ret))
            self.TrackedObject=None
            self.trackedId=-1
            self.TrackedObjectLife=0
            self.TrackedObjectName="unknow"
            
        return self.trackedId
    
    def _imcrop(self,img, bbox): 
        x1,y1,x2,y2 = bbox
        if x1 < 0 or y1 < 0 or x2 > img.shape[1] or y2 > img.shape[0]:
            img, x1, x2, y1, y2 = self._pad_img_to_fit_bbox(img, x1, x2, y1, y2)
        return img[y1:y2, x1:x2, :]

    def _pad_img_to_fit_bbox(self,img, x1, x2, y1, y2):
        img = np.pad(img, ((np.abs(np.minimum(0, y1)), np.maximum(y2 - img.shape[0], 0)),
                   (np.abs(np.minimum(0, x1)), np.maximum(x2 - img.shape[1], 0)), (0,0)), mode="constant")
        y1 += np.abs(np.minimum(0, y1))
        y2 += np.abs(np.minimum(0, y1))
        x1 += np.abs(np.minimum(0, x1))
        x2 += np.abs(np.minimum(0, x1))
        return img, x1, x2, y1, y2    
    
    def setFaceReconnitionTrackedObjectId(self,pFace,pImage,pImgOrigin):
                #pObjects.getObjectsInImage(self.img)
        trackedObject=self.getTrackedObject()
        if self.trackedId==-1 or (len(self.trackedObjectOccurency)>0 and self.trackedObjectOccurency['all'] >150):
            self.trackedObjectOccurency={}
            
        if (trackedObject is not None):
            (x,y,w,h)=trackedObject
            yMargin=int( (h*(0.2)) ) #Add 20%
            xMargin=int( (w*(0.1)) ) #Add 10%
            #crop_img=self._imcrop(pImgOrigin,(max(x-xMargin,0),max(y-yMargin,0),min(x+w+xMargin,640),min(y+h+yMargin,480)))
            crop_img = pImgOrigin[max(y-yMargin,0):min(y+h+yMargin,480), max(x-xMargin,0):min(x+w+xMargin,640)]
            #print ("crop_img (x1="+str(max(x-xMargin,0))+",y="+str(max(y-yMargin,0))+",x2="+str(min(x+w+xMargin,640))+",y2="+str(min(y+h+yMargin,480))+")")
            #cv2.imshow("Face detection", crop_img)          
            
            #cv2.imshow("", crop_img)
        
            #faceRecognition = FaceRecognition()
            
            #perform a prediction
            if (self.faceRecognition is not None):
                self.faceRecognition.predict(crop_img)
            if (self.faceRecognition is not None):
                aTrackedObjectName=self.faceRecognition.getLabelText();
            else:
                aTrackedObjectName=""
            
            #Set ObjectName based on occurence
            if len(self.trackedObjectOccurency)>0:
                if aTrackedObjectName not in self.trackedObjectOccurency: 
                    self.trackedObjectOccurency[aTrackedObjectName]=1 
                else: 
                    self.trackedObjectOccurency[aTrackedObjectName]=self.trackedObjectOccurency[aTrackedObjectName]+1
                self.trackedObjectOccurency['all']=self.trackedObjectOccurency['all']+1
            else:
                self.trackedObjectOccurency[aTrackedObjectName]=1
                self.trackedObjectOccurency['all']=1
        
            TrackedObjectNameList = sorted(self.trackedObjectOccurency.items(), key=operator.itemgetter(1), reverse=True)
            #print("Sorted TrackedObjectNameList="+str(TrackedObjectNameList))
                
            if (len(TrackedObjectNameList)>2):
                aTrackedObjectName=(TrackedObjectNameList[2])[0]
                if aTrackedObjectName=='unknow':
                    aTrackedObjectName=(TrackedObjectNameList[1])[0]
            elif (len(TrackedObjectNameList)==1):
                aTrackedObjectName='unknow'
            elif (len(TrackedObjectNameList)>2):
                aTrackedObjectName=(TrackedObjectNameList[1])[0]
               
            if self.trackedObjectOccurency['all'] >20 :
                self.TrackedObjectName=aTrackedObjectName
            else:
                self.TrackedObjectName='searching...('+str(self.trackedObjectOccurency['all'])+')'

            #display both images
            #cv2.imshow("Recognition", cv2.resize(predicted_img1, (400, 500)))
        return self.trackedId
            
    def getTrackedObjectIdCamShift(self,pFace,pImage):
        hsv = cv2.cvtColor(pImage, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0,180], 1)
        #ret, track_window = cv2.meanShift(dst, pFace, self.term_crit)
        output= cv2.CamShift(dst, pFace, self.term_crit)
        ret, track_window =output
        self.TrackedObject = track_window
        (x,y,w,h)=track_window
        cv2.rectangle(pImage,(x,y),(x+w,y+h),(0,0,255),2)
        
        pts = cv2.boxPoints(ret)
        pts = np.int0(pts)
        cv2.polylines(pImage,[pts],True, 255,2)

import os
import pickle
class Face2(RobotObject):
    def __init__(self):
        RobotObject.__init__(self)
        self.objects=array
        self.TrackedObject=None
        self.TrackedObjectLife=0
        self.trackedId=-1
        self.TrackedObjectName="unknow"
        self.image = None
        self.SearchFrequency=0
        
        # load our serialized face detector from disk
        print("[INFO] loading face detector...")
        protoPath = os.path.sep.join(["resources/Other/face_detection_model", "deploy.prototxt"])
        modelPath = os.path.sep.join(["resources/Other/face_detection_model",  "res10_300x300_ssd_iter_140000.caffemodel"])
        self.detector = cv2.dnn.readNetFromCaffe(protoPath, modelPath)
        
        # load our serialized face embedding model from disk
        print("[INFO] loading face recognizer...")
        self.embedder = cv2.dnn.readNetFromTorch("resources/Other/face_detection_model/openface_nn4.small2.v1.t7")
        
        # load the actual face recognition model along with the label encoder
        self.recognizer = pickle.loads(open("resources/Other/face_detection_model/recognizer.pickle", "rb").read())
        self.le = pickle.loads(open("resources/Other/face_detection_model/le.pickle", "rb").read())
        
    def _displayTrackingInfo(self,pImg,pOrigin):
        #Draw centered marker
        cv2.rectangle(pImg,(int(320)-50,int(240)-50),(int(320)+50,int(240)+50),(0,255,0),2)
        cv2.drawMarker(pImg, (int(320),int(240)), (0,255,0),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)      

        return self
 
    def _searchObject(self,pImg,pOrigin):
        ## (1) prepare data
        area=None
        bigestArea=None
        
        frame = imutils.resize(pOrigin, width=600)
        (h, w) = pOrigin.shape[:2]
            
        # construct a blob from the image
        imageBlob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0), swapRB=False, crop=False)
    
        # apply OpenCV's deep learning-based face detector to localize
        # faces in the input image
        self.detector.setInput(imageBlob)
        detections = self.detector.forward()
        
        # loop over the detections
        for i in range(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the prediction
            confidence = detections[0, 0, i, 2] 
            
            # filter out weak detections
            if confidence > 0.5:
                # compute the (x, y)-coordinates of the bounding box for
                # the face
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
    
                # extract the face ROI
                face = frame[startY:endY, startX:endX]
                (fH, fW) = face.shape[:2]
    
                # ensure the face width and height are sufficiently large
                if fW < 20 or fH < 20:
                    continue     

                # construct a blob for the face ROI, then pass the blob
                # through our face embedding model to obtain the 128-d
                # quantification of the face
                faceBlob = cv2.dnn.blobFromImage(face, 1.0 / 255,
                    (96, 96), (0, 0, 0), swapRB=True, crop=False)
                self.embedder.setInput(faceBlob)
                vec = self.embedder.forward()
    
                # perform classification to recognize the face
                preds = self.recognizer.predict_proba(vec)[0]
                j = np.argmax(preds)
                proba = preds[j]
                name = self.le.classes_[j]
    
                # draw the bounding box of the face along with the
                # associated probability
                text = "{}: {:.2f}%".format(name, proba * 100)
                y = startY - 10 if startY - 10 > 10 else startY + 10
                cv2.rectangle(pImg, (startX, startY), (endX, endY), (255, 255, 255), 2)
                cv2.putText(pImg, text, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)
                
                #Get biggest box
                area=(startX,startY,endY,endX)
                if (bigestArea is None):
                    bigestArea=area
                    self.TrackedObjectName=text
                    #self.trackedId=random.randint(1,10000)
                else:
                    (x,y,w,h)=area
                    (a,b,c,d)=bigestArea
                    if ((w*h) > (c*d)):
                        bigestArea=area
                        self.TrackedObjectName=text
                        #self.trackedId=random.randint(1,10000)
                (bx, by, bheight,bwidth)=bigestArea
                cv2.rectangle(pImg, (bx, by), (bwidth, bheight), (0, 0, 255), 4)
                cv2.drawMarker(pImg, (int(bx+(bwidth/2)),int(by+(bheight/2))), (255,255,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)
           
        return bigestArea,pImg
    
    def _displayTrackedObject(self,pImg,pOrigin):
        pts=self.getTrackedObject()

        if (pts is not None):
            #Display text in top left corner
            bx, by, bwidth, bheight = pts#cv2.boundingRect(pts)
            cv2.putText(pImg, ('Tracked (Id='+str(self.trackedId)+')'), (bx,by), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv2.LINE_AA)
            cv2.drawMarker(pImg, (int(bx+(bwidth/2)),int(by+(bheight/2))), (255,255,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)

        return self

class Boards(RobotObject):
    def __init__(self):
        RobotObject.__init__(self)
        self.objects=array
        self.TrackedObject=None
        self.TrackedObjectLife=0
        self.trackedId=-1
        self.TrackedObjectName="unknow"
        self.image = None
        
    def _displayTrackingInfo(self,pImg,pOrigin):
        #Draw centered marker
        cv2.rectangle(pImg,(int(320)-50,int(240)-50),(int(320)+50,int(240)+50),(0,255,0),2)
        cv2.drawMarker(pImg, (int(320),int(240)), (0,255,0),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)      

        return self
 
    def _searchObject(self,pImg,pOrigin):
        ## (1) prepare data
            
        MIN_MATCH_COUNT = 10
        
        area=None
        #imgname1 = "resources/Boards/salon_emma_smal1.jpg"
        imgname1 = "resources/Boards/salon_mer_smal.png"
        img1 = cv2.imread(imgname1)
        img2 = pOrigin.copy()
        canvas = pOrigin.copy()
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        ## (2) Create SIFT object
        sift = cv2.xfeatures2d.SIFT_create()
        
        ## (3) Create flann matcher
        matcher = cv2.FlannBasedMatcher(dict(algorithm = 1, trees = 5), {})
        
        ## (4) Detect keypoints and compute keypointer descriptors
        kpts1, descs1 = sift.detectAndCompute(gray1,None)
        kpts2, descs2 = sift.detectAndCompute(gray2,None)
        
        if (descs1 is None or descs2 is None):
            return None,canvas
        ## (5) knnMatch to get Top2
        matches = matcher.knnMatch(descs1, descs2, 2)
        # Sort by their distance.
        matches = sorted(matches, key = lambda x:x[0].distance)
        
        ## (6) Ratio test, to get good matches.
        good = [m1 for (m1, m2) in matches if m1.distance < 0.7 * m2.distance]
        
        ## (7) find homography matrix
        if len(good)>MIN_MATCH_COUNT:
            print( "Enough matches are found:"+str(len(good)))
            ## (queryIndex for the small object, trainIndex for the scene )
            src_pts = np.float32([ kpts1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kpts2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            ## find homography matrix in cv2.RANSAC using good match points
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            #matchesMask2 = mask.ravel().tolist()
            h,w = img1.shape[:2]
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            if (M is None): return None,pImg
            
            area = cv2.perspectiveTransform(pts,M)
            cv2.polylines(pImg,[np.int32(area)],True,(0,0,255),3, cv2.LINE_AA)
            bx, by, bwidth, bheight = cv2.boundingRect(area)
            if (bx<0): bwidth=bwidth+bx; bx=0;
            if (by<0): bheight=bheight+by; by=0;
            area=(bx, by, bwidth, bheight)
            (x,y,w,h)=area
            area=(int(x-x*0.2),int(y-y*0.2),int(w+w*0.2),int(h+h*0.2))
            
            # compute polygone around detected area
            cv2.rectangle(pImg,(bx,by),(bx+bwidth,by+bheight),(0,0,255),2)
            
            """
            ## (8) drawMatches
            matched = cv2.drawMatches(img1,kpts1,canvas,kpts2,good,None)#,**draw_params)
            
            ## (9) Crop the matched region from scene
            h,w = img1.shape[:2]
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            perspectiveM = cv2.getPerspectiveTransform(np.float32(dst),pts)
            found = cv2.warpPerspective(img2,perspectiveM,(w,h))
            
            ## (10) save and display
            cv2.imwrite("matched.png", matched)
            cv2.imwrite("found.png", found)
            cv2.imshow("matched", matched);
            #cv2.imshow("found", found);
            #cv2.waitKey();cv2.destroyAllWindows()"""
        else:
            print( "Not enough matches are found - {}/{}".format(len(good),MIN_MATCH_COUNT)) 
            
        return area,pImg
    
    def _displayTrackedObject(self,pImg,pOrigin):
        pts=self.getTrackedObject()
        
        #Display text in top left corner
        bx, by, bwidth, bheight = pts#cv2.boundingRect(pts)
        cv2.putText(pImg, ('Tracked (Id='+str(self.trackedId)+')'), (bx,by), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv2.LINE_AA)
        cv2.drawMarker(pImg, (int(bx+(bwidth/2)),int(by+(bheight/2))), (255,255,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)

        return self
    
                     
class Boards1(RobotObject):
    def __init__(self):
        RobotObject.__init__(self)
        self.objects=array
        self.TrackedObject=None
        self.TrackedObjectLife=0
        self.trackedId=-1
        self.TrackedObjectName="unknow"
        self.image = None

    
    def _SearchObject(self,pImg,pOrigin):
           ## (1) prepare data
            
        MIN_MATCH_COUNT = 10
        
        area=None
        #imgname1 = "resources/Boards/salon_emma_smal1.jpg"
        imgname1 = "resources/Boards/salon_mer_smal.png"
        img1 = cv2.imread(imgname1)
        img2 = pOrigin.copy()
        canvas = pOrigin.copy()
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        ## (2) Create SIFT object
        sift = cv2.xfeatures2d.SIFT_create()
        
        ## (3) Create flann matcher
        matcher = cv2.FlannBasedMatcher(dict(algorithm = 1, trees = 5), {})
        
        ## (4) Detect keypoints and compute keypointer descriptors
        kpts1, descs1 = sift.detectAndCompute(gray1,None)
        kpts2, descs2 = sift.detectAndCompute(gray2,None)
        
        if (descs1 is None or descs2 is None):
            return None,canvas
        ## (5) knnMatch to get Top2
        matches = matcher.knnMatch(descs1, descs2, 2)
        # Sort by their distance.
        matches = sorted(matches, key = lambda x:x[0].distance)
        
        ## (6) Ratio test, to get good matches.
        good = [m1 for (m1, m2) in matches if m1.distance < 0.7 * m2.distance]
        
        ## (7) find homography matrix
        if len(good)>MIN_MATCH_COUNT:
            print( "Enough matches are found:"+str(len(good)))
            ## (queryIndex for the small object, trainIndex for the scene )
            src_pts = np.float32([ kpts1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kpts2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            ## find homography matrix in cv2.RANSAC using good match points
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            #matchesMask2 = mask.ravel().tolist()
            h,w = img1.shape[:2]
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            if (M is None): return None,pImg
            
            area = cv2.perspectiveTransform(pts,M)
            cv2.polylines(pImg,[np.int32(area)],True,(0,0,255),3, cv2.LINE_AA)
            bx, by, bwidth, bheight = cv2.boundingRect(area)
            if (bx<0): bwidth=bwidth+bx; bx=0;
            if (by<0): bheight=bheight+by; by=0;
            area=(bx, by, bwidth, bheight)
            (x,y,w,h)=area
            area=(int(x-x*0.2),int(y-y*0.2),int(w+w*0.2),int(h+h*0.2))
            
            # compute polygone around detected area
            cv2.rectangle(pImg,(bx,by),(bx+bwidth,by+bheight),(0,0,255),2)
            
            """
            ## (8) drawMatches
            matched = cv2.drawMatches(img1,kpts1,canvas,kpts2,good,None)#,**draw_params)
            
            ## (9) Crop the matched region from scene
            h,w = img1.shape[:2]
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            dst = cv2.perspectiveTransform(pts,M)
            perspectiveM = cv2.getPerspectiveTransform(np.float32(dst),pts)
            found = cv2.warpPerspective(img2,perspectiveM,(w,h))
            
            ## (10) save and display
            cv2.imwrite("matched.png", matched)
            cv2.imwrite("found.png", found)
            cv2.imshow("matched", matched);
            #cv2.imshow("found", found);
            #cv2.waitKey();cv2.destroyAllWindows()"""
        else:
            print( "Not enough matches are found - {}/{}".format(len(good),MIN_MATCH_COUNT)) 
            
        return area,pImg   
    
    def _displayTrackedObject(self,pImg,pOrigin):
        return self
    
    def getObjectsInImage(self,pImg,pOrigin):
        
        if (pImg is None):
            self.image=None
            return self
        #self.objects,pImg = self._getBoard(pImg,pOrigin)
        #Step1: detect Board
        #print("self.TrackedObjectLife="+str(self.TrackedObjectLife)+"TrackedObject="+str(self.getTrackedObject()))            
       
        self.TrackedObjectLife=self.TrackedObjectLife+1
        
        if self.TrackedObjectLife % 20==0:
            self.setTrackedObject(None)
            self.trackedId=-1
            if self.getTrackedObject() is None  :
                self.objects,pImg = self._SearchObject(pImg,pOrigin)
                self.setTrackedObject(self.objects)
            
        if self.TrackedObjectLife>=100:
            self.TrackedObjectLife=0
                 

        #Step2: Track Board
        if self.getTrackedObject() is not None:
            if self.trackedId == -1:
                self.initTrackedObjectId(self.getTrackedObject(),pImg,pOrigin)
                self.trackedId="Salon"
            else:
                self.getTrackedObjectId(self.getTrackedObject(),pImg,pOrigin)
     
        #Step3: Display Tracked okject identification
        if (self.getTrackedObject() is not None):
            pts=self.getTrackedObject()
            
            #Display text in top left corner
            bx, by, bwidth, bheight = pts#cv2.boundingRect(pts)
            cv2.putText(pImg, ('Tracked (Id='+str(self.trackedId)+')'), (bx,by), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2, cv2.LINE_AA)
            cv2.drawMarker(pImg, (int(bx+(bwidth/2)),int(by+(bheight/2))), (255,255,255),markerType=cv2.MARKER_CROSS, markerSize=30, thickness=2, line_type=cv2.LINE_AA)


            #Display Sharp
            #cv2.polylines(pImg,[np.int32(pts)],True,(255,0,255),3, cv2.LINE_AA)
            #Display the center of the Sharp
            """M1 = cv2.moments(pts)
            (cX,cY)=(0,0)
            if M1["m00"]!=0:
                cX = int(M1["m10"] / M1["m00"])
            if M1["m00"]!=0:   
                cY = int(M1["m01"] / M1["m00"])
            cv2.circle(pImg, (cX, cY), 7, (255, 255, 255), -1)"""    
        
        # split our image color_space into blue, green, red components
        #r,g,b = cv2.split(pImg)

        # compose our image back but this time as red, green and blue
        #pImg= cv2.merge([r,g,b])
        #self.image=pImg
        return self
    
    def setTrackedObject(self,pFace):
        self.TrackedObject=pFace

    def initTrackedObjectId(self,pFace,pImage,pImgOrigin):
        # Create mask and normalized histogram
        (x,y,w,h)=pFace
        pFace=(int(x-(x*0.2)/2),int(y-(y*0.2)/2),int(w+w*0.2),int(h+h*0.2))
        (x,y,w,h)=pFace
        roi = pImgOrigin[y:y+h, x:x+w]
        self.hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv_roi, np.array((0., 30.,32.)), np.array((180.,255.,255.)))
        self.roi_hist = cv2.calcHist([self.hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1)
        if (self.trackedId == -1):
            self.trackedId=random.randint(1,10000)
        return self.trackedId
    
    def getTrackedObjectId(self,pFace,pImage,pImgOrigin):
        hsv = cv2.cvtColor(pImgOrigin, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0,180], 1)
        #https://docs.opencv.org/3.4.0/db/df8/tutorial_py_meanshift.html
        ret, track_window = cv2.meanShift(dst, pFace, self.term_crit)
        (x,y,w,h)=track_window
        self.TrackedObject = track_window
        cv2.rectangle(pImage,(x,y),(x+w,y+h),(255,0,255),2)
        if ret > 15:
            self.TrackedObject=None
            self.trackedId=-1
            self.TrackedObjectLife=0
            self.TrackedObjectName="unknow"
        return self.trackedId
        
    def initTrackedObjectId1(self,pFace,pImage,pImgOrigin):
        # Create mask and normalized histogram
        (c,r,w,h)=pFace
        roi = pImgOrigin[r:r+h, c:c+w]
        self.hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv_roi, np.array((0., 30.,32.)), np.array((180.,255.,255.)))
        self.roi_hist = cv2.calcHist([self.hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1)

        if (self.trackedId == -1):
            self.trackedId=random.randint(1,10000)
        return self.trackedId
    
    def getTrackedObjectId1(self,pFace,pImage,pImgOrigin):
        hsv = cv2.cvtColor(pImgOrigin, cv2.COLOR_BGR2HSV)
        
        #Evaluate 
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0,180], 1)
        #https://docs.opencv.org/3.4.0/db/df8/tutorial_py_meanshift.html
        ret, track_window = cv2.meanShift(dst, pFace, self.term_crit)
        self.TrackedObject = track_window
        (x,y,w,h)=track_window
        cv2.rectangle(pImage,(x,y),(x+w,y+h),(0,255,255),2)

        if ret > 15:
            print ("Lost tracking, ret="+str(ret))
            self.TrackedObject=None
            self.trackedId=-1
            self.TrackedObjectLife=0
            self.TrackedObjectName="unknow"
        return self.trackedId
    
    def initTrackedObjectId2(self,pFace,pImage,pImgOrigin):
        # Create mask and normalized histogram
        #(y,x,w,h)=pFace
        #roi = pImage[x:x+h, y:y+w]
        self.hsv_roi = cv2.cvtColor(pImage, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(self.hsv_roi, np.array((0., 60.,32.)), np.array((180.,255.,255.)))
        self.roi_hist = cv2.calcHist([self.hsv_roi],[0],mask,[180],[0,180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        self.trackedId=1000;
        return self.trackedId
    
    def getTrackedObjectId2(self,pFace,pImage,pImgOrigin):
        if (pFace is None):
            self.initTrackedObjectId(pFace,pImage,pImgOrigin)
            
        img_gray = cv2.cvtColor(pImgOrigin, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([img_gray], [0], self.roi_hist, [0,180], 1)
        bx, by, bwidth, bheight = cv2.boundingRect(pFace)
        pFace=(bx, by, bwidth, bheight)
        ret, track_window = cv2.CamShift(dst, pFace, self.term_crit)
        
        self.TrackedObject = cv2.boxPoints(ret)
        self.TrackedObject = np.int0(self.TrackedObject)
        cv2.polylines(pImage,[self.TrackedObject],True, 255,2)
        
        #Display the center of the Sharp
        M1 = cv2.moments(self.TrackedObject)
        (cX,cY)=(0,0)
        if M1["m00"]!=0:
            cX = int(M1["m10"] / M1["m00"])
        if M1["m00"]!=0:   
            cY = int(M1["m01"] / M1["m00"])
        cv2.circle(pImage, (cX, cY), 7, (255, 255, 255), -1)  
        """
        if ret is None or ret > 15:
            self.TrackedObject=None
            self.trackedId=-1
            self.TrackedObjectLife=0"""

class RobotWebCam(object):
    def __init__(self,name,type,path,host='',port=''):
        self.host=host
        self.port=port
        self.type=type
        self.path=path
        self.filecapture=''
        self.count=0

    def _get_image(self):
        if self.type== 'StreamMpeg':
            self.url = 'http://'+self.host+':'+str(self.port)+self.path
            self.img=self._get_image_streammpeg()
        elif self.type== 'streamPicture':
            self.url ='http://'+self.host+':'+str(self.port)+self.path
            self.img=self._get_image_streampicture()
        elif self.type== 'file':
            self.url =self.path
            self.img=self._get_image_filempeg()
        elif self.type== 'picture':
            self.url =self.path
            self.img=cv2.imread(self.url)
            
        if (self.img is not None):
            equ = cv2.equalizeHist(self.img)
            self.img = np.hstack((self.img,equ)) #stacking images side-by-side
            self.imgOrigine=(self.img).copy()
            #cv2.imshow("image", self.img);
            #cv2.imshow("origine", self.imgOrigine);
        return self.img
        
    def _get_image_streampicture(self):
        # Get our image from the phone
        #imgResp = urllib.urlopen(self.url)
        with urlopen(self.url) as url:
            imgResp = url.read()

        # Convert our image to a numpy array so that we can work with it
        imgNp = np.array(bytearray(imgResp),dtype=np.uint8)

        # Convert our image again but this time to opencv format
        img = cv2.imdecode(imgNp,-1)

        return img
 
    def _get_image_filempeg(self):
        if (self.filecapture==''):
            self.filecapture = cv2.VideoCapture(self.url)
        
        if(self.filecapture.isOpened()):
            ret, img = self.filecapture.read()
        
        return img

    def _get_image_streammpeg2(self):
        img =None
        stream = urllib.request.urlopen(self.url)
        chunk =stream.read(1024)
        bytes += chunk
        a = bytes.find(b'\xff\xd8')
        b = bytes.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes[a:b+2]
            bytes = bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        return img
   
    def _get_image_streammpeg(self):
        # https://stackoverflow.com/questions/21702477/how-to-parse-mjpeg-http-stream-from-ip-camera
        # http://www.virtualroadside.com/blog/index.php/2015/04/03/better-python-interface-to-mjpg-streamer-using-opencv/
        
        # mjpg-streamer URL
        stream = urlopen(self.url)
            
        # Read the boundary message and discard
        stream.readline()
        
        sz = 0
        rdbuffer = None
        
        clen_re = re.compile(b'Content-Length: (\d+)\\r\\n')
        
        # Read each frame
        # TODO: This is hardcoded to mjpg-streamer's behavior
              
        stream.readline()                    # content type
        
        try:                                 # content length
            m = clen_re.match(stream.readline()) 
            clen = int(m.group(1))
        except:
            exit
        
        stream.readline()                    # timestamp
        stream.readline()                    # empty line
        
        # Reallocate buffer if necessary
        if clen > sz:
            sz = clen*2
            rdbuffer = bytearray(sz)
            rdview = memoryview(rdbuffer)
        

        # Read frame into the preallocated buffer
        stream.readinto(rdview[:clen])
        
        stream.readline() # endline
        stream.readline() # boundary
            
        # This line will need to be different when using OpenCV 2.x
        img = cv2.imdecode(np.frombuffer(rdbuffer, count=clen, dtype=np.byte), flags=cv2.IMREAD_COLOR)

        return img


    def _get_image_string(self,img):
        if (img is None):
            return None
        # return the image as a string, but also give out its shape(width,height) and color_space
        return (img.tostring(), img.shape[1::-1], 'RGB')

    
    def readImg(self):
        self.img=self._get_image();
        return self
    
    def getPygameImage(self):
        # Get the image
        img = self.img
        
        if (img is None):
            return None
        
        # get our image in string format and also the size and color_space for pygame to Use
        img,shape,color_space = self._get_image_string(img)

        # create the pygame image from the string, size and color space
        img = pygame.image.frombuffer(img,shape,color_space)

        return img,shape
    
    def processObjects(self,pObjectsFace='',pObjectsBoard=''):
        if pObjectsFace =='':
            pObjectsFace=Faces()
        
        #sonarValues=[(45,20,20),(90,40,40)]
        #robotGUI    = RobotGUI()
        #robotGUI.ShowSonarScan(img,sonarValues)'
        
        # compose our image back but this time as red, green and blue
        self.img = pObjectsFace.getObjectsInImage(self.img,self.imgOrigine)
        cv2.imshow("Record_processObjects", self.imgOrigine)
        self.img = pObjectsBoard.getObjectsInImage(self.img,self.imgOrigine)
        #self.img = self.findObjects(self.img)
        #self.img = self.findContours(self.img)

        return self
    
    def findObjects(self,pImage):
            ## (1) prepare data
            
        MIN_MATCH_COUNT = 10
        
        imgname1 = "resources/Boards/salon_emma_smal1.jpg"
        img1 = cv2.imread(imgname1)
        img2 = pImage.copy()
        canvas = pImage.copy()
        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        
        
        ## (2) Create SIFT object
        sift = cv2.xfeatures2d.SIFT_create()
        
        ## (3) Create flann matcher
        matcher = cv2.FlannBasedMatcher(dict(algorithm = 1, trees = 5), {})
        
        ## (4) Detect keypoints and compute keypointer descriptors
        kpts1, descs1 = sift.detectAndCompute(gray1,None)
        kpts2, descs2 = sift.detectAndCompute(gray2,None)
        
        if (descs1 is None or descs2 is None):
            return canvas
        ## (5) knnMatch to get Top2
        matches = matcher.knnMatch(descs1, descs2, 2)
        # Sort by their distance.
        matches = sorted(matches, key = lambda x:x[0].distance)
        
        ## (6) Ratio test, to get good matches.
        good = [m1 for (m1, m2) in matches if m1.distance < 0.7 * m2.distance]
        
        ## (7) find homography matrix
        if len(good)>MIN_MATCH_COUNT:
            print( "Enough matches are found:"+str(len(good)))
            ## (queryIndex for the small object, trainIndex for the scene )
            src_pts = np.float32([ kpts1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kpts2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            ## find homography matrix in cv2.RANSAC using good match points
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            #matchesMask2 = mask.ravel().tolist()
            h,w = img1.shape[:2]
            pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            if (M is None): return pImage
            dst = cv2.perspectiveTransform(pts,M)
            
            # compute polygone around detected area
            cv2.polylines(canvas,[np.int32(dst)],True,(0,255,0),3, cv2.LINE_AA)
            
            # compute the center of the polygone
            M1 = cv2.moments(dst)
            (cX,cY)=(0,0)
            if M1["m00"]!=0:
                cX = int(M1["m10"] / M1["m00"])
            if M1["m00"]!=0:   
                cY = int(M1["m01"] / M1["m00"])
            cv2.circle(canvas, (cX, cY), 7, (255, 255, 255), -1)
            print ("Center=("+str(cX)+","+str(cY)+")")
            
            ## (8) drawMatches
            #matched = cv2.drawMatches(img1,kpts1,canvas,kpts2,good,None)#,**draw_params)
            
            ## (9) Crop the matched region from scene
            #h,w = img1.shape[:2]
            #pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            #dst = cv2.perspectiveTransform(pts,M)
            #perspectiveM = cv2.getPerspectiveTransform(np.float32(dst),pts)
            #found = cv2.warpPerspective(img2,perspectiveM,(w,h))
            
            ## (10) save and display
            #cv2.imwrite("matched.png", matched)
            #cv2.imwrite("found.png", found)
            #cv2.imshow("matched", matched);
            #cv2.imshow("found", found);
            #cv2.waitKey();cv2.destroyAllWindows()
        else:
            print( "Not enough matches are found - {}/{}".format(len(good),MIN_MATCH_COUNT))
            
        return canvas    
    
    
    def findContours(self,pImage):
        gray = cv2.cvtColor(pImage, cv2.COLOR_BGR2GRAY)
        
        ret,thresh = cv2.threshold(gray,127,255,1)
        
        _ , contours, val = cv2.findContours(thresh,1,2)
        
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            if len(approx)==5:
                cv2.drawContours(pImage,[cnt],0,255,-1)
            elif len(approx)==3:
                cv2.drawContours(pImage,[cnt],0,(0,255,0),-1)
            elif len(approx)==4:
                cv2.drawContours(pImage,[cnt],0,(0,0,255),-1)
            elif len(approx) == 9:
                cv2.drawContours(pImage,[cnt],0,(255,255,0),-1)
            elif len(approx) > 15:
                cv2.drawContours(pImage,[cnt],0,(0,255,255),-1)
        
        return pImage
    
    def recordImage(self,pFaces):
        crop_img=pFaces._getFacesWithEyesInImage(self.imgOrigine)
        if (crop_img is not None):
            cv2.imshow("Record2.1", crop_img)
            cv2.imwrite("resources/Faces/unknows/record"+str(random.randint(1,10000))+"_image.png", crop_img)
            print("Face recored")
        else:
            face_cascade = cv2.CascadeClassifier('C:\opt\opencv\sources\data\haarcascades\haarcascade_frontalface_alt2.xml')
            crop_img=face_cascade.detectMultiScale(self.imgOrigine, 1.3, 5)
            if (crop_img is not None):
                cv2.imshow("Record2.2", crop_img)
                cv2.imwrite("resources/Faces/unknows/record"+str(random.randint(1,10000))+"_image.png", crop_img)
                print(" ========= Face recored =======")           
        return self
    
    def displayInfo(self,pStart,pIsTracking,pBoard):
        if (self.img is not None):
            """area = self.img[200:460,0:300]
            cv2.blur(area, (30,30))
            cv2.rectangle(self.img,(0,300),(200,460),(255,255,255),2)"""
            
            end = time.time()
            seconds = end - pStart
            fps  = 1 / seconds;
            
            height, width = self.img.shape[:2]
            info="Fps="+str(format(fps, '.2f'))+"\nTracking="+str(pIsTracking)+"\nLocalization="+str(pBoard.trackedId)
            y0, dy = height-30, 15
            for i, info in enumerate(info.split('\n')):
                y = y0 - i*dy
                cv2.putText(self.img, info, (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
        return self

class Ball2(RobotObject):
    def __init__(self):
        RobotObject.__init__(self)
        self.greenLower = None
        self.greenUpper = None
        self.SearchFrequency=0
        self.mask=None
        #self.greenLower = (29, 86, 6)
        #self.greenUpper = (64, 255, 255)
        
        self.greenLower=(0,0,167)
        self.greenUpper=(179,255,255)

             
    def _searchObject(self,pImg,pOrigin):
        # Blur the frame, and convert it to the HSV color space
        
        blurred = cv2.GaussianBlur(pOrigin, (11, 11), 1)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask       
             
        mask = cv2.inRange(hsv, self.greenLower, self.greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #initDisplaymask()
        cv2.imshow('Applied Mask',mask)
        #(self.greenLower,self.greenUpper)=displayMask(mask,self.greenLower,self.greenUpper)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
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
    
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                cv2.circle(pImg, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(pImg, center, 5, (0, 0, 255), -1)
                #return (int(x-radius), int(y-radius),int(radius),int(radius)),pImg
                area=(int(x-radius),int(y-radius),int(radius)*2,int(radius)*2)
                return area,pImg
            
        #area,pImg
        return None,pImg

    def getObjectsInImage2(self,pImg,pImgOrigin):
        
        #Check parameters
        if (pImg is None or pImgOrigin is None):
            self.image=None
            return self
        
        self._displayTrackingInfo(pImg,pImgOrigin)
        
        self.objects,pImg = self._searchObject(pImg,pImgOrigin)
        self.setTrackedObject(self.objects)
        if self.getTrackedObject() is not None:
            self._displayTrackedObject(pImg,pImgOrigin)
                
        return self    
    
class Ball(RobotObject):
    def __init__(self):
        RobotObject.__init__(self)
        self.resizeWidth=350 #img resize to improve perf (but decrease capacity to find circle or color window)
        self.lowerColor = None
        self.upperColor = None
        self.SearchFrequency=0
        #self.lowerColor = (29, 86, 6)
        #self.upperColor = (64, 255, 255)
        self.circle=None
        self.radius=None

             
    def _searchObject(self,pImg,pOrigin):
        
        #If no lowerColor or upperColor found Then find circle and associated self.lowerColor self.upperColor
        if (self.lowerColor is None or self.upperColor is None):
            self.circle=self.getCircle(pOrigin)
            if (self.circle is not None):
                (self.lowerColor,self.upperColor)=self.getColorsFromCircle(pOrigin,self.circle)
        
        if (self.lowerColor is None or self.upperColor is None):
            return None,pImg
        
        # Blur the frame, and convert it to the HSV color space
        
        blurred = cv2.GaussianBlur(pOrigin, (11, 11), 1)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask       
             
        mask = cv2.inRange(hsv, self.lowerColor, self.upperColor)
        #mask = cv2.erode(mask, None, iterations=2)
        #mask = cv2.dilate(mask, None, iterations=2)
        #initDisplaymask()
        cv2.imshow('Applied Mask',mask)
        #(self.LowerColor,self.greenUpper)=displayMask(mask,self.LowerColor,self.greenUpper)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        center = None
    
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / max(M["m00"],1)), int(M["m01"] / max(M["m00"],1)))
    
            # only proceed if the radius meets a minimum size
            if radius > 10:
                self.radius=radius
                # draw the circle and centroid on the frame,
                cv2.circle(pImg, (int(x), int(y)), int(radius),(0, 255, 255), 2)
                cv2.circle(pImg, center, 5, (0, 0, 255), -1)
                #return (int(x-radius), int(y-radius),int(radius),int(radius)),pImg
                area=(int(x-radius),int(y-radius),int(radius)*2,int(radius)*2)
                return area,pImg
            else:
                self.radius=None
            
        #area,pImg
        return None,pImg

    def getCircle(self,pImg):
        
        working_img = imutils.resize(pImg, width=self.resizeWidth)
        
        gray = cv2.cvtColor(working_img, cv2.COLOR_BGR2GRAY)
        circles=None
        for i in range(15,10,-1):
            current_circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, i/10, 100)
            if (current_circles is None):
                break
            if (len(current_circles>0)):
                
                circles_looop=np.round(current_circles[0, :]).astype("int")
                for (x, y, r) in circles_looop:
                    #print("loop=",i/10,"r=",r)
                    if (r>0):
                        circles=current_circles
            
        r_max=0
        max_circle=None
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
        
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                if (r>r_max):
                    r_max=r
                    max_circle=(x, y, r)
                    #print("Found circle r=",r)
        return max_circle
        
    
    def getColorsFromCircle(self,pImg,pCircle):
        lower,upper= None,None
        
        working_img = imutils.resize(pImg, width=self.resizeWidth)
        good_lower=None
        good_upper=None
        
        if (pCircle is not None):
            
            (x, y, r)=pCircle
            
            height,width = working_img.shape[:2]
            size_img=height*width
            maskBall = np.zeros((height,width), np.uint8)
            
            cv2.circle(maskBall, (x, y), r,(255,255,255),thickness=-1)
            img_onlyBall = cv2.bitwise_and(working_img, working_img, mask=maskBall)
            cv2.imshow("2 - Image Balle seulement",img_onlyBall)
            
            maskNoBall = cv2.bitwise_not(maskBall)
            img_noBall = cv2.bitwise_and(working_img, working_img, mask=maskNoBall)
            #cv2.imshow("2 - Image sans Balle",img_noBall)
            
            #Step 3: Detect lower and upper colors in disk image
            #---------------------------------------------------
            
            #Methode 1: Detect lower and upper colors in the Disk mask
            hsvRoi = cv2.cvtColor(working_img, cv2.COLOR_BGR2HSV)
            img_mask = hsvRoi[np.where(maskBall == 255)] 
            img_avg = np.mean(img_mask, axis=0) 
            #lower = np.min(img_mask, axis=0) 
            #upper = np.max(img_mask, axis=0) 
            max_DeltaPercent=0
            max_colors=None
            max_withBallPercent=0
            max_withoutBallPercent=0
            
            for x in range(0, 255):
                good_lower=lower
                good_upper=upper
                lower=img_avg-[x,x,x]
                upper=img_avg+[x,x,x]
                
                lh,lv,lc=lower
                if (lh<0 or lv<0 or lc<0):
                    break
                
                hh,hv,hc=upper
                if (hh>255 or hv>255 or hc>255):
                    break
            
                #Get no ball mask
                hsv = cv2.cvtColor(img_noBall, cv2.COLOR_BGR2HSV)
                maskNoBall_FromColor = cv2.inRange(hsv, lower, upper)
                n_white_pix_noBall = np.sum(maskNoBall_FromColor == 255) 
                #print('n_white_pix_noBall[',x,']=:', n_white_pix_noBall)
                cv2.imshow("3.1 - Masque - maskNoBall_FromColor", maskNoBall_FromColor)
                
                #Get ball mask
                hsv = cv2.cvtColor(img_onlyBall, cv2.COLOR_BGR2HSV)
                maskBall_FromColor = cv2.inRange(hsv, lower, upper)
                n_white_pix_ball = np.sum(maskBall_FromColor == 255) 
                #print('n_white_pix_Ball[',x,']=:', n_white_pix_ball)
                cv2.imshow("3.2 - Masque - maskBall_FromColor", maskBall_FromColor)
                
                withBallPercent=n_white_pix_ball*100/size_img
                withoutBallPercent=n_white_pix_noBall*100/size_img
                
                #print("Step={},withBallPercent={:.1f}%, withoutBallPercent={:.1f}%, delta={:.2f}%".format(x,withBallPercent,withoutBallPercent,withBallPercent-withoutBallPercent))
                
                deltaPercent=withBallPercent-withoutBallPercent
                #If deltaPercent is increasing update max_DeltaPercent
                if max_DeltaPercent<deltaPercent:
                    max_DeltaPercent=deltaPercent
                    max_withBallPercent=withBallPercent
                    max_withoutBallPercent=withoutBallPercent
                    
                #If deltaPercent is de-increasing or deltaPercent > max_DeltaPercent
                if (deltaPercent<max_DeltaPercent or deltaPercent<0):
                    
                    #If There is no enouht point detected for the ball
                    if (withBallPercent<5 or deltaPercent<0):
                        return (None,None)
                    break
                
            print('Min HSV = {}\nMax HSV = {}\nAvg HSV = {}\nmax_DeltaPercent={:.1f}%\nmax_withBallPercent={:.1f}%,max_withoutBallPercent={:.1f}%'.format(good_lower, good_upper,img_avg,max_DeltaPercent,max_withBallPercent,max_withoutBallPercent))
        return (good_lower,good_upper)

    def getObjectsInImage2(self,pImg,pImgOrigin):
        
        #Check parameters
        if (pImg is None or pImgOrigin is None):
            self.image=None
            return self
        
        self._displayTrackingInfo(pImg,pImgOrigin)
        
        self.objects,pImg = self._searchObject(pImg,pImgOrigin)
        self.setTrackedObject(self.objects)
        if self.getTrackedObject() is not None:
            self._displayTrackedObject(pImg,pImgOrigin)
                
        return self 
         

if __name__ == '__main__':
    print('No main in this class')