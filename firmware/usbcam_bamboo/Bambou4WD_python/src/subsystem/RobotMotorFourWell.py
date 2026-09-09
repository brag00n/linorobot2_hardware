'''
Created on 22 janv. 2018

@author: OLIVIERCousinier
'''

from subsystem._RobotSubSystem import _RobotSubSystem
from subsystem.RobotWebCamMotorized import RobotWebCamMotorized


class RobotMotorFourWell(_RobotSubSystem):
    def __init__(self,pHmi,pCommRobot,pRobotWebCamMotorized):
        _RobotSubSystem.__init__(self, pHmi,pCommRobot)
        self.speedMax=10
        self.speedMin=0
        self.speed=0
        self.speedOld=-1
        
        self.deltaMin=-10
        self.deltaMax=10
        self.delta=0
        self.deltaOld=-1
        
        self.RobotWebCamMotorized=pRobotWebCamMotorized
        self.moveToCenterCameraLoop=0
        return
    
    def setSpeed(self,pSpeed):
        self.speed=pSpeed
        return self
    
    def getSpeed(self):
        return self.speed 
    
    def setDelta(self,pDelta):
        self.delta=pDelta
        return self
    
    def getDelta(self):
        return self.delta 
    
    def move(self):
        if (self.speed>self.speedMax):
            self.speed=self.speedMax
        if (self.speed<self.speedMin):
            self.speed=self.speedMin
            
        if (self.delta>self.deltaMax):
            self.delta=self.deltaMax
        if (self.delta<self.deltaMin):
            self.delta=self.deltaMin
            
        if (self.speed+self.delta>0):
            speed=self.speed+self.delta
        else:
            speed=0
            
        if (self.speedOld!=self.speed or self.deltaOld!=self.delta):
            if (self.speed==0):
                self.comCommand.sendWithResponse(bytes([255,0,0,int(speed),255]))
            else:
                if (self.speed+self.delta<self.speedMin):
                    self.delta=self.delta+1
                if (self.speed+self.delta>self.speedMax):
                    self.delta=self.delta-1
                    
                if (self.delta==0):         
                    #Set Left speed
                    self.comCommand.sendWithResponse(bytes([255,2,1,int(self.speed),255]))
                    #Set Right speed            
                    self.comCommand.sendWithResponse(bytes([255,2,2,int(self.speed),255]))
                    #Set MOTOR_GO_FORWARD
                    self.comCommand.sendWithResponse(bytes([255,0,1,int(self.speed),255]))   
                elif (self.delta>0):
                    #Set Left speed
                    self.comCommand.sendWithResponse(bytes([255,2,1,int(speed),255]))
                    #Set Right speed            
                    self.comCommand.sendWithResponse(bytes([255,2,2,int(self.speed+self.delta),255]))
                    #Set MOTOR_GO_FORWARD
                    self.comCommand.sendWithResponse(bytes([255,0,1,int(self.speed),255]))   
                elif (self.delta<0):
                    #Set Left speed
                    self.comCommand.sendWithResponse(bytes([255,2,1,int(self.speed+self.delta),255]))
                    #Set Right speed            
                    self.comCommand.sendWithResponse(bytes([255,2,2,int(speed),255]))
                    #Set MOTOR_GO_FORWARD
                    self.comCommand.sendWithResponse(bytes([255,0,1,int(self.speed),255]))   
                    
                self.speedOld=self.speed
                
    def moveToCenterCamera(self):
        if (self.RobotWebCamMotorized is None):
            return
        
        self.moveToCenterCameraLoop=self.moveToCenterCameraLoop+1
        print("self.moveToCenterCameraLoop="+str(self.moveToCenterCameraLoop)+", self.moveToCenterCameraLoop%20="+str(self.moveToCenterCameraLoop%20))
                    
        if (self.moveToCenterCameraLoop==5):
            self.moveToCenterCameraLoop=0
            horiz=self.RobotWebCamMotorized.getAngleH()
            speed=3
            
            if (horiz>100):
                self.comCommand.send(bytes([255,2,1,speed,255]))
                self.comCommand.send(bytes([255,2,2,speed,255]))
                self.comCommand.send(bytes([255,0,4,0,255]))  
                self.RobotWebCamMotorized.setAngleH(self.RobotWebCamMotorized.getAngleH()-8).moveHorizontal()
                print("Sent order TURN LEFT")  
            elif (horiz<80):
                self.comCommand.send(bytes([255,2,1,speed,255]))
                self.comCommand.send(bytes([255,2,2,speed,255]))
                self.comCommand.send(bytes([255,0,3,0,255]))
                print("Sent order TURN RIGHT")    
                self.RobotWebCamMotorized.setAngleH(self.RobotWebCamMotorized.getAngleH()+8).moveHorizontal()
            
            speed=self.getSpeed()
            self.setSpeed(0).move()
            self.setSpeed(0).move()
            self.setSpeed(0).move()
            self.setSpeed(speed)
        return