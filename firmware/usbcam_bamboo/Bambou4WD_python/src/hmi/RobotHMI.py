'''
Created on 21 janv. 2018

@author: OLIVIERCousinier
'''
import pygame
import time
import cv2

class RobotHMI(object):
    def __init__(self):
        self.img=None
        self.textInfo=None
        self.textDebug=None
        #pygame data
        self.clock       = pygame.time.Clock()
        self.screen      = pygame.display.set_mode((1,1))
        self.pygameImage = None
        self.time=time
        self.timerStart=0
        self.count=0
        self.fps=0
        self.resolution=None
        return
    
    def start(self):
        self.timerStart=time.time()
    
    def setInfo(self,pText):
        self.textInfo=pText
        return self
    
    def setDebug(self,pText):
        self.textDebug=pText
        return self
    
    def setImage(self,pImage):
        self.img=pImage
        return self

    def _getImagePygame(self,pImage):
        # get our image in string format and also the size and color_space for pygame to Use
        img,shape,color_space = (pImage.tostring(), pImage.shape[1::-1], 'RGB')

        # create the pygame image from the string, size and color space
        img = pygame.image.frombuffer(img,shape,color_space)
                # split our image color_space into blue, green, red components
        self.pygameImage=img
        return img,shape
    
    
    def display(self):
        
        if (self.img is not None):
            """area = self.img[200:460,0:300]
            cv2.blur(area, (30,30))
            cv2.rectangle(self.img,(0,300),(200,460),(255,255,255),2)"""
            
            seconds = time.time() - self.timerStart
            
            if (seconds>0):
                fps  = 1 / seconds;            
                self.fps=fps
            
            if (self.textInfo is not None):
                textInfo="Fps="+str(format(self.fps, '.2f'))+"\n"+str(self.textInfo)
            else:
                textInfo="Fps="+str(format(self.fps, '.2f'))
                
            height, width = self.img.shape[:2]
            y0, dy = height-30, 15
            for i, info in enumerate((textInfo).split('\n')):
                y = y0 - i*dy
                cv2.putText(self.img, info, (10,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                
            img,shape=self._getImagePygame(self.img)
            if (self.resolution is None):
                self.resolution= shape[:2]
                width,height = self.resolution
                self.screen=pygame.display.set_mode((width,height))
            self.screen.blit(img,(0,0))
            #self.time.sleep(0.005)
            pygame.display.flip()
            self.clock.tick(0)
        return