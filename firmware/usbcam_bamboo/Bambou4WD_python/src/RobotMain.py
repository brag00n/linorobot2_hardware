'''
Created on 21 janv. 2018

@author: OLIVIERCousinier
'''

import sys, traceback
from communication.RobotCommunication import RobotCom,RobotComSerial,RobotComUrl
from subsystem.RobotWebCamMotorized import RobotWebCamMotorized
from subsystem.RobotMotorFourWell import RobotMotorFourWell
from hmi.RobotHMI import RobotHMI,pygame
from interaction.FaceRecognition import FaceRecognition
from interaction.RobotObject import Faces,Face2,Boards,Ball
from interaction.RobotObject import Ground


def find_between(s, start, end):
    output=""
    try:
        output=(s.split(start))[1].split(end)[0]
    except Exception as ex:
        template = "An exception of type {0} occurred. Arguments:\n{1!r}"
        message = template.format(type(ex).__name__, ex.args)
        print (message)
    return output

#robotWebCam             =RobotComUrl('Robot video file','file','resources/Samples/young-business-team.mp4');robotSerial            =RobotCom("Robot Fake com",'192.168.1.1',2001);   
#robotWebCam             =RobotComUrl('Robot video file','file','resources/Samples/business-people-walking2.mp4');robotSerial            =RobotCom("Robot Fake com",'192.168.1.1',2001);   
#robotWebCam             =RobotComUrl('Robot video file','file','resources/Samples/VID_20160822_185142895.mp4');robotSerial            =RobotCom("Robot Fake com",'192.168.1.1',2001);
#robotWebCam             =RobotComUrl('Robot video file','file','resources/Samples/VID_20140610_195238.3gp');robotSerial            =RobotCom("Robot Fake com",'192.168.1.1',2001);
#robotWebCam             =RobotComUrl('Robot video file','file','C:\\Users\\OLIVIERCousinier\\Videos\\PetitJournal.mp4');robotSerial            =RobotCom("Robot Fake com",'192.168.1.1',2001);
robotWebCam             =RobotComUrl('Robot video file','file mp4','resources/Samples/ball_tracking_example.mp4');robotSerial            =RobotCom("Robot Fake com",'192.168.1.1',2001);

#robotWebCam             = RobotComUrl('Robot PC webCam','webcam');robotSerial             =RobotCom("Robot Fake com",'192.168.1.1',2001)
#robotWebCam             = RobotComUrl('Robot video Cam','StreamMpeg','/?action=stream','192.168.1.1',8080);robotSerial             =RobotComSerial("Robot command",'192.168.1.1',2001)
#robotWebCam             =RobotComUrl('External IPWebCam','streamPicture','/shot.jpg','192.168.0.10',8080);robotSerial             =RobotComSerial("Robot command",'192.168.1.1',2001)
#robotWebCam             =RobotComUrl('External IPWebCam','streamPicture','/shot.jpg','192.168.43.1',8080);robotSerial             =RobotCom("Robot command",'192.168.1.1',2001)
#robotWebCam             =RobotComUrl('Robot Ispy','streamPicture','/','192.168.100.1',9876);robotSerial             =RobotCom("Robot command",'192.168.1.1',2001)

robotHmi                =RobotHMI();
robotWebCamMotorized    =RobotWebCamMotorized(robotHmi,robotSerial,robotWebCam)
#faces                   =Faces(FaceRecognition())
faces                   =Faces();
boards                   =Boards();
ball                   =Ball();
robotMotorFourWell      =RobotMotorFourWell(robotHmi,robotSerial,robotWebCamMotorized)

run         = True
isTrackingFace    = False
isSensor    = False
isGround    = False
isMoveToCenterCamera = False
sensorsData = ""
oldEventType= None
pygame.init()
pygame.key.set_repeat(50, 50)
robotMotorFourWell.setSpeed(3)

try:
        
    run=True
    previousEventKey=None
    curentEventKey=None
    
    robotWebCamMotorized.setAngleH(100).moveHorizontal()
    robotWebCamMotorized.setAngleV(20).moveVertical()
    
    while run:
        robotHmi.start()
        robotWebCamMotorized.readImage()
           
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                print("Bye bye from quit")
              
            if event.type == pygame.KEYDOWN:
                curentEventKey=event.key
                
                if event.key == pygame.K_ESCAPE:
                    run=False
                    print("Exit with ESCAPE")

                if event.key == pygame.K_s:
                    robotWebCamMotorized.setAngleH(robotWebCamMotorized.getAngleH()+1).moveHorizontal()
                if event.key == pygame.K_f:
                    robotWebCamMotorized.setAngleH(robotWebCamMotorized.getAngleH()-1).moveHorizontal()
                if event.key == pygame.K_e:
                    robotWebCamMotorized.setAngleV(robotWebCamMotorized.getAngleV()+1).moveVertical()
                if event.key == pygame.K_d:
                    robotWebCamMotorized.setAngleV(robotWebCamMotorized.getAngleV()-1).moveVertical()
                    
                if event.key == pygame.K_UP and event.key!=previousEventKey:
                    #robotMotorFourWell.setSpeed(robotMotorFourWell.getSpeed()+1).move()
                    robotSerial.send(bytes([255,2,1,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,2,2,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,0,2,0,255]))
                    print("Sent order K_e Go Front Speed=",robotMotorFourWell.getSpeed())
                if event.key == pygame.K_DOWN and event.key!=previousEventKey:
                    #robotMotorFourWell.setSpeed(robotMotorFourWell.getSpeed()-1).move()
                    robotSerial.send(bytes([255,2,1,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,2,2,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,0,1,0,255]))
                    print("Sent order K_c Go Back Speed=",robotMotorFourWell.getSpeed())
                if event.key == pygame.K_RIGHT and event.key!=previousEventKey:
                    #robotMotorFourWell.setDelta(robotMotorFourWell.getDelta()-1).move()
                    robotSerial.send(bytes([255,2,1,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,2,2,1,255]))
                    robotSerial.send(bytes([255,0,2,0,255]))
                    print("Sent order K_s Go Left SpeedDelta=",robotMotorFourWell.getDelta())
                if event.key == pygame.K_LEFT and event.key!=previousEventKey:
                    #robotMotorFourWell.setDelta(robotMotorFourWell.getDelta()+1).move()
                    robotSerial.send(bytes([255,2,1,1,255]))
                    robotSerial.send(bytes([255,2,2,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,0,2,0,255]))
                    print("Sent order K_f Go Right SpeedDelta=",robotMotorFourWell.getDelta())
                if event.key == pygame.K_PAGEUP and event.key!=previousEventKey:
                    #robotMotorFourWell.setDelta(robotMotorFourWell.getDelta()+1).move()
                    #Go Right
                    robotSerial.send(bytes([255,2,1,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,2,2,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,0,4,0,255]))
                    print("Sent order K_f Go Right SpeedDelta=",robotMotorFourWell.getDelta())
                if event.key == pygame.K_PAGEDOWN and event.key!=previousEventKey:
                    #robotMotorFourWell.setDelta(robotMotorFourWell.getDelta()+1).move()
                    #Go Left
                    robotSerial.send(bytes([255,2,1,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,2,2,robotMotorFourWell.getSpeed(),255]))
                    robotSerial.send(bytes([255,0,3,0,255]))
                    print("Sent order K_f Go Right SpeedDelta=",robotMotorFourWell.getDelta())                       
                    
                if event.key == pygame.K_SPACE:
                    robotMotorFourWell.setSpeed(0).move()
                    print("Sent order K_d Stop SpeedDelta=",robotMotorFourWell.getDelta())
                
                    
                if event.key == pygame.K_F1:
                    isTrackingFace=not isTrackingFace
                    ball=None
                    ball=Ball()
                    print("Change isTrackingFace status=",isTrackingFace)
                    
                if event.key == pygame.K_F2:
                    isSensor=not isSensor
                    print("Change sensor status=",isSensor)
                                        
                if event.key == pygame.K_F3:
                    isRecord=True
                    robotWebCamMotorized.recordImage(faces)
                    
                if event.key == pygame.K_F4:
                    robotWebCamMotorized.setAngleH(100).moveHorizontal()
                    robotWebCamMotorized.setAngleV(20).moveVertical()
                
                if event.key == pygame.K_F5:
                    isGround=not isGround
                    
                if event.key == pygame.K_F6:
                    isMoveToCenterCamera=not isMoveToCenterCamera
                    
                if   event.key == pygame.K_1: robotMotorFourWell.setSpeed(1)
                elif event.key == pygame.K_2: robotMotorFourWell.setSpeed(2)
                elif event.key == pygame.K_3: robotMotorFourWell.setSpeed(3)
                elif event.key == pygame.K_4: robotMotorFourWell.setSpeed(4)
                elif event.key == pygame.K_5: robotMotorFourWell.setSpeed(5)
                elif event.key == pygame.K_6: robotMotorFourWell.setSpeed(6)
                elif event.key == pygame.K_7: robotMotorFourWell.setSpeed(7)
                elif event.key == pygame.K_8: robotMotorFourWell.setSpeed(8)
                elif event.key == pygame.K_9: robotMotorFourWell.setSpeed(9)
                elif event.key == pygame.K_0: robotMotorFourWell.setSpeed(0)
            else:
                speed=robotMotorFourWell.getSpeed()
                robotMotorFourWell.setSpeed(0).move()
                robotMotorFourWell.setSpeed(0).move()
                robotMotorFourWell.setSpeed(0).move()
                robotMotorFourWell.setSpeed(speed)
                #print("Stop, curentEventKey="+str(curentEventKey)+"previousEventKey="+str(previousEventKey)+"event.type="+str(event.type))


        if (isTrackingFace):
            robotWebCamMotorized.trackObject(faces,boards,ball)
            
        if (isMoveToCenterCamera and isTrackingFace):
            robotMotorFourWell.moveToCenterCamera()
            
        if (isGround):
            ground                   =Ground();
            ground.detectGround(robotWebCamMotorized.getImage(),robotWebCamMotorized.getImageOrigine())
            
        if isSensor==True:
            data=robotSerial.sendWithResponse(bytes([255,19,5,0,255]))
            distanceFront= find_between(data, "[distanceFront=", ",")
            distanceLeft=find_between(data, "distanceLeft=", ",")
            distanceRight=find_between(data, "distanceRight=", ",")
            Left_Speed_Hold=find_between(data, "Left_Speed_Hold=", ",")
            Right_Speed_Hold=find_between(data, "Right_Speed_Hold=", ",")
            sensorsData="distanceFront="+distanceFront+"\n"+"distanceLeft="+distanceLeft+"\n"+"distanceRight="+distanceRight+"\n"+"Left_Speed_Hold="+Left_Speed_Hold+"\n"+"Right_Speed_Hold="+Right_Speed_Hold+"\n"

              
        #robotWebCam.processObjects(faces)
        if robotWebCamMotorized.getImage() is not None:
            robotHmi.setImage(robotWebCamMotorized.getImage())
            robotHmi.setInfo(sensorsData+"\nF1:TrackingFace="+str(isTrackingFace)+"\nF2:Sensor="+str(isSensor)+"\nF6:MoveToCenterCamera="+str(isMoveToCenterCamera)+"\nSpeed="+str(robotMotorFourWell.getSpeed()))
            robotHmi.display()
            
    robotSerial.close()
    robotWebCam.close()
    pygame.quit()
    
except Exception as ex:
    template = "An exception of type {0} occurred.\nArguments:\n{1!r}"
    message = template.format(type(ex).__name__, ex.args)
    print (message)
    print (traceback.format_exc())
    print("Falal Error, exiting...")
    exit
finally:
    robotSerial.close()
    robotWebCam.close()
    pygame.quit()
    sys.exit()
    print("Bye bye")
    
    