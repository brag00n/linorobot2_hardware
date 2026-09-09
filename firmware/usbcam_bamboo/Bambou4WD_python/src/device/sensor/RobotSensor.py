'''
Created on 21 janv. 2018

@author: OLIVIERCousinier
'''
import cv2
from urllib.request import urlopen
import numpy as np
import re
import random
import imutils
import argparse

class RobotSensor(object):
    def __init__(self,communication):
        self.com=communication
        return
    
class RobotSensorWebCam(RobotSensor):
    
    def __init__(self,pCommunication):
        RobotSensor.__init__(self, pCommunication)
        self.image=None
        self.imageOrigine=None
        self.filecapture=None
        self.camera=None
        self.vs=None
        return
    
    def readImage(self):
        if self.com.type== 'StreamMpeg':
            self.com.url = 'http://'+self.com.host+':'+str(self.com.port)+self.com.path
            self.image=self._get_image_streammpeg()
        elif self.com.type== 'streamPicture':
            self.com.url ='http://'+self.com.host+':'+str(self.com.port)+self.com.path
            self.image=self._get_image_streampicture()
        elif self.com.type== 'file':
            self.com.url =self.com.path
            self.image=self._get_image_filempeg()
        elif self.com.type== 'file mp4':
            self.com.url =self.com.path
            self.image=self._get_image_filempeg4()
        elif self.com.type== 'webcam':
            if (self.camera is None):
                self.camera = cv2.VideoCapture(0)
            (grabbed, self.image) = self.camera.read()
            if (not grabbed):
                self.image=None
                self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)
            
        if (self.image is not None):
            # split our image color_space into blue, green, red components
            self.image = imutils.resize(self.image, width=600)
            if (self.com.type == 'webcam' or self.com.type == 'streamPicture'):
                b,g,r = cv2.split(self.image)
        
                # compose our image back but this time as red, green and blue
                self.image= cv2.merge([r,g,b])
            self.imageOrigine=(self.image).copy()
        else:
            self.image=None
            self.imageOrigine=None
                

        return self
    
    def recordImage(self,pFaces):
        crop_img=pFaces._getFacesWithEyesInImage(self.imageOrigine)
        if (crop_img is not None):
            cv2.imshow("Record3.1", crop_img)
            cv2.imwrite("resources/Faces/unknows/record"+str(random.randint(1,10000))+"_image.png", crop_img)
            print("Face recored")
        else:
            face_cascade = cv2.CascadeClassifier('resources/Other/Classifier/Haar/haarcascade_frontalface_alt2.xml')
            crop_imgs=face_cascade.detectMultiScale(self.imageOrigine, 1.3, 5)
            for (x,y,w,h) in crop_imgs:
                roi_color = self.imageOrigine[y:y+h, x:x+w]
                if (roi_color is not None):
                    cv2.imshow("Record3.2", roi_color)
                    cv2.imwrite("resources/Faces/unknows/record"+str(random.randint(1,10000))+"_image.png", roi_color)
                    print(" ========= Face recored =======")                              
        return self
        
    def _get_image_streampicture(self):

        imgResp=self.com.receive();

        # Convert our image to a numpy array so that we can work with it
        imgNp = np.array(bytearray(imgResp),dtype=np.uint8)

        # Convert our image again but this time to opencv format
        img=None
        if (imgNp is not None and imgNp.size >0):
                img = cv2.imdecode(imgNp,-1)

        return img
 
    def _get_image_filempeg4(self): 
        if (self.vs is None):
            ap = argparse.ArgumentParser()
            args = vars(ap.parse_args())
            args["video"]=self.com.url
            self.vs = cv2.VideoCapture(args["video"])
        frame = self.vs.read()
        return frame[1]
    
    def _get_image_filempeg(self):
        img = None
        if (self.filecapture is None):
            self.filecapture = cv2.VideoCapture(self.com.url)
        
        if(self.filecapture.isOpened()):
            ret, img = self.filecapture.read()
            
            if (img is not None):
                b,g,r = cv2.split(img)
        
                # compose our image back but this time as red, green and blue
                img= cv2.merge([r,g,b])
        
        return img

           
    def _get_image_streammpeg(self):
        # mjpg-streamer URL
        stream = urlopen(self.com.url)
        #print(self.com.url)
            
        # Read the boundary message and discard
        stream.readline()
        
        sz = 0
        rdbuffer = None
        rdview=None
        
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
        if (rdview is None):  return None
        
        stream.readinto(rdview[:clen])
        
        stream.readline() # endline
        stream.readline() # boundary
            
        # This line will need to be different when using OpenCV 2.x
        img = cv2.imdecode(np.frombuffer(rdbuffer, count=clen, dtype=np.byte), flags=cv2.IMREAD_COLOR)
        
        if (img is not None):
            b,g,r = cv2.split(img)

            # compose our image back but this time as red, green and blue
            img= cv2.merge([r,g,b])

        return img

    def getImage(self):
        return self.image

    def getImageOrigine(self):
        return self.imageOrigine
    