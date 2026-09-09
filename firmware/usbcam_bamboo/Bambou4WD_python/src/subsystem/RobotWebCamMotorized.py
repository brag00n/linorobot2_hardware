'''
Created on 22 janv. 2018

@author: OLIVIERCousinier
'''
import math
import cv2
from lib.python_pid import python_PID 
#from Bamboo4W.hmi.RobotHMI import RobotHMI
from device.sensor.RobotSensor import RobotSensorWebCam
from interaction.RobotObject import Ground
#from Bamboo4W.device.motion.RobotServoMotor import RobotServoMotor
from subsystem._RobotSubSystem import _RobotSubSystem 

class RobotWebCamMotorized(_RobotSubSystem):
    
    def __init__(self,pHmi,pCommRobot,pComRobotWebCam):
        _RobotSubSystem.__init__(self, pHmi,pCommRobot)
        self.sensorWebCam=RobotSensorWebCam(pComRobotWebCam)
        self.imageString=None
        self.angleH=90
        self.angleV=20
        self.angleHOld=90
        self.angleVOld=20
        
        self.angleHMin=9
        self.angleHMax=178
        self.angleVMin=20
        self.angleVMax=40
        
        self.pid_x=None
        self.image=None
        return
    
    def readImage(self):
        self.setImage(self.sensorWebCam.readImage().getImage())
        return self
    
    def getImage(self):
        return self.image
    
    def setImage(self, pImage):
        self.image=pImage
    
    def getImageOrigine(self):
        return self.sensorWebCam.getImageOrigine()
    
    def moveToTrackedArea2(self, pTrakedArea):
        
        if pTrakedArea is None:
            #self.pid_x=None
            return (self.angleH, self.angleV)
        
        (x, y, w, h) = pTrakedArea
        center_x = x + (w / 2)
        center_y = y + (h / 2)
        step=2
        kp = 0.2
        ki = 0
        kd = 0
        P_ON_E="P_ON_E" #set mode "Porportionnal on Error" (and not "Porportionnal on Mesure" )

        if (self.pid_x is  None):
            self.pid_x = python_PID(center_x,self.angleH,320    , kp, ki, kd, P_ON_E,True) #Input, Output, Consigne (setPoint), kp, ki, kd, Proportional&, controler direction
            self.pid_x.SetOutputLimits(70, 110); #only move between 60° and 120° on PID output 
            self.pid_x.SetMode("AUTOMATIC")
            
        self.pid_x.setInput(center_x)
        if (self.pid_x.Compute()):
            angle_h=(math.ceil(self.pid_x.GetOutput()))
            #self.setAngleH(math.ceil(self.angleH+self.pid_x.GetOutput())).moveHorizontal()
            print("===== Old AngleH="+str(self.angleH)+" ,new AngleH="+str(math.ceil(self.pid_x.GetOutput()))+" ,delta="+str(self.pid_x.GetOutput()))        
        """ 
        if center_x - 320 > 50:
            step=self.mapStep(center_x - 320)
            if (self.angleH - step >= 1):
                self.angleH = self.angleH - step
            self.moveHorizontal()
        elif center_x - 320 < -50:
            step=self.mapStep(center_x - 320)
            if (self.angleH + step <= 180):
                self.angleH = self.angleH + step
            self.moveHorizontal() 
        """ 
            
        if center_y - 240 > 50:
            step=self.mapStep(center_y - 240)
            if (self.angleV - step >= 20):
                self.angleV = self.angleV - step
            self.moveVertical()
        elif center_y - 240 < -50:
            step=self.mapStep(center_y - 240)
            if (self.angleV + step <= 40):
                self.angleV = self.angleV + step
            self.moveVertical() 
        
        #if ( not (self.angleHOld==self.angleH and self.angleVOld==self.angleV)):
        #print("trackedArea=", pTrakedArea, "center_x=", center_x, "center_y=", center_y, ",deviationX=", center_x - 320,",deviationY=", center_y - 240, "(angleX=", self.angleH, "pAngleY=", self.angleV,")")         
        self.angleHOld=self.angleH
        self.angleVOld=self.angleV        
        return self
 
    def moveToTrackedArea(self, pTrakedArea):
        
        isUpdated=False
        if pTrakedArea is None:
            return (self.angleH, self.angleV)
        
        (x, y, w, h) = pTrakedArea
        center_x = x + (w / 2)
        center_y = y + (h / 2)
        step=2
        
        # If center_x is on the right go to the left
        if center_x - 320 > 50:
            step=self.mapStep(center_x - 320)
            if (self.angleH - step >= 1):
                self.angleH = self.angleH - step
                isUpdated=True
            self.moveHorizontal()
        # If center_x is on the left go to the right
        elif center_x - 320 < -50:
            step=self.mapStep(center_x - 320)
            if (self.angleH + step <= 180):
                self.angleH = self.angleH + step
                isUpdated=True
            self.moveHorizontal()  
            
        # If center_y is on the up go down
        if center_y - 240 > 50:
            step=self.mapStep(center_y - 240)
            if (self.angleV - step >= 20):
                self.angleV = self.angleV - step
                isUpdated=True
            self.moveVertical()
        # If center_y is on the back go up
        elif center_y - 240 < -50:
            step=self.mapStep(center_y - 240)
            if (self.angleV + step <= 40):
                self.angleV = self.angleV + step
                isUpdated=True
            self.moveVertical() 
        
        """
        # If angleH or angleV has been updated, display it (tracked objet is not in the tracking area)
        if (isUpdated):
            print("trackedArea=", pTrakedArea, "center_x=", center_x, "center_y=", center_y, ",deviationX=", center_x - 320,",deviationY=", center_y - 240, "(angleX=", self.angleH, "pAngleY=", self.angleV,")")         
        """
        return self
       
    def mapStep(self,delta):
        delta=int(math.fabs(delta))
        if delta <=0 and delta > 20:
            return 1
        if delta >=20 and delta > 50:
            return 2
        if delta >=50:
            return 3

    def recordImage(self,pFaces):
        return self.sensorWebCam.recordImage(pFaces)
        
    def trackObject(self,pObjectsFace,pObjectsBoard,pObjectsBall):
        #Step1: find tracked object
        #self.setImage(pObjectsFace.getObjectsInImage(self.getImage(),self.getImageOrigine()).getImage())
        #self.setImage(pObjectsBoard.getObjectsInImage(self.getImage(),self.getImageOrigine()).getImage())
        self.setImage(pObjectsBall.getObjectsInImage(self.image,self.getImageOrigine()).getImage())

        #Step2: move camera to tracked object 
        if (pObjectsFace.getTrackedObject() is not None):
            self.moveToTrackedArea(pObjectsFace.getTrackedObject())
        elif (pObjectsBoard.getTrackedObject() is not None):
            self.moveToTrackedArea(pObjectsBoard.getTrackedObject())
        elif (pObjectsBall.getTrackedObject() is not None):
            self.moveToTrackedArea(pObjectsBall.getTrackedObject())
            
        return self
    
    def getAngleH(self):
        return self.angleH
    
    def setAngleH(self,pAngle):
        self.angleH=pAngle
        return self
    
    def getAngleV(self):
        return self.angleV
    
    def setAngleV(self,pAngle):
        self.angleV=pAngle
        return self

    def moveHorizontal(self):
        if (self.angleH>self.angleHMax):
            self.angleH=self.angleHMax
        if (self.angleH<self.angleHMin):
            self.angleH=self.angleHMin
            
        if (self.angleHOld!=self.angleH):
            self.comCommand.send(bytes([255,1,7,self.getAngleH(),255]))
            self.angleHOld=self.angleH
        return
    
    def moveVertical(self):
        if (self.angleV>self.angleVMax):
            self.angleV=self.angleVMax
        if (self.angleV<self.angleVMin):
            self.angleV=self.angleVMin
            
        if (self.angleVOld!=self.angleV):
            self.comCommand.send(bytes([255,1,8,self.getAngleV(),255]))
            self.angleVOld=self.angleV
        return