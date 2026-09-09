"""*********************************************************************************************
 * Arduino PID Library - Version 1.2.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under the MIT License
 *********************************************************************************************"""

# see https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.cpp

import time

AUTOMATIC   = "AUTOMATIC"
MANUAL      = "MANUAL"
REVERSE     = "REVERSE"
P_ON_E      = "P_ON_E"

class python_PID:
    """PID Controller
    """
    
    """Constructor (...)*********************************************************
     *    The parameters specified here are those for for which we can't set up
     *    reliable defaults, so we need to have the user set them.
     **************************************************************************"""
    
    def __init__ (self,pInput, pOutput, pSetpoint, pKp, pKi, pKd, pPOn, pControllerDirection):
        
        self.startTime=0
        self.curentTime=0
        
        self.kp = pKp;
        self.ki = pKi;
        self.kd = pKd;
   
        self.myOutput = pOutput;
        self.myInput = pInput;
        self.lastInput = pInput
        self.mySetpoint = pSetpoint;
        self.inAuto = False;
    
        self.SetOutputLimits(0, 255);           #default output limit corresponds to
                                                #  the arduino pwm limits
        self.SampleTime = 100;                  #default Controller Sample Time is 0.1 seconds
    
        self.SetControllerDirection(pControllerDirection);
        self.SetTuningsInit(pKp, pKi, pKd, pPOn);
    
        self.lastTime = self.millis() - self.SampleTime;
        self.outputSum=0
        
    
    """ Compute() **********************************************************************
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns True when the output is computed,
     *   False when nothing has been done.
     *********************************************************************************"""
     
    def Compute(self):
 
        if (not self.inAuto): 
            return False
        
        now = self.millis()
        timeChange = (now - self.lastTime);
        if (timeChange>=self.SampleTime):

            """Compute all the working error variables"""
            theInput = self.myInput;
            error = self.mySetpoint - theInput;
            theDInput = (theInput - self.lastInput)
            self.outputSum = self.outputSum+(self.ki * error)
            
            """Add Proportional on Measurement, if P_ON_M is specified"""
            if(not self.pOnE): 
                self.outputSum= self.outputSum - (self.kp * theDInput)
            
            if(self.outputSum > self.outMax): 
                self.outputSum= self.outMax;
            elif (self.outputSum < self.outMin): 
                self.outputSum= self.outMin;
        
            """Add Proportional on Error, if P_ON_E is specified"""
            output=0;
            if(self.pOnE): 
                output = self.kp * error;
        
            """Compute Rest of PID Output"""
            output = output + self.outputSum - self.kd * theDInput;
        
            if(output > self.outMax):
                output = self.outMax;
            elif (output < self.outMin):
                output = self.outMin;
            self.myOutput = output;
            
            """Remember some variables for next time"""
            self.myInput=theInput;
            self.lastInput = theInput;
            self.lastTime = now
            return True;
        else: 
            return False;

    """ SetTunings(...)*************************************************************
     * This function allows the controller's dynamic performance to be adjusted.
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation
     *****************************************************************************"""
    def SetTuningsInit(self,pKp, pKi, pKd, pPOn):
        if (pKp<0 or pKi<0 or pKd<0): 
            return;
        
        self.pOn = pPOn;
        self.pOnE = (pPOn == P_ON_E);
        
        self.dispKp = pKp; self.dispKi = pKi; self.dispKd = pKd;
        
        self.SampleTimeInSec = (self.SampleTime)/1000;
        self.kp = pKp;
        self.ki = pKi * self.SampleTimeInSec;
        self.kd = pKd / self.SampleTimeInSec;
        
        if(self.controllerDirection == REVERSE):
            self.kp = (0 - self.kp);
            self.ki = (0 - self.ki);
            self.kd = (0 - self.kd);

    
    """ SetTunings(...)*************************************************************
     * Set Tunings using the last-rembered POn setting
     *****************************************************************************"""
    def SetTunings(self,pKp,pKi,pKd):
        self.SetTuningsInit(pKp, pKi, pKd, self.pOn); 
    
    """ Setself.SampleTime(...) *********************************************************
     * sets the period, in Milliseconds, at which the calculation is performed
     *****************************************************************************"""
    def SetSampleTime(self, pNewSampleTime):

        if (pNewSampleTime > 0):
            ratio  = pNewSampleTime  / self.SampleTime;
            self.ki = self.ki * ratio;
            self.kd = self.kd / ratio;
            self.SampleTime = time.time();

    
    """ SetOutputLimits(...)****************************************************
     *     This function will be used far more often than SetInputLimits.  while
     *  the input to the controller will generally be in the 0-1023 range (which is
     *  the default already,)  the output will be a little different.  maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     *************************************************************************"""
    def SetOutputLimits(self, pMin, pMax):
        if(pMin >= pMax):
            return
        self.outMin = pMin;
        self.outMax = pMax;
    
        if(self.inAuto):
            if(self.myOutput > self.outMax):
                self.myOutput = self.outMax;
            elif (self.myOutput < self.outMin):
                self.myOutput = self.outMin;
            
            if(self.outputSum > self.outMax):
                self.outputSum= self.outMax
            elif(self.outputSum < self.outMin):
                self.outputSum= self.outMin;
    
    """ SetMode(...)****************************************************************
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     *****************************************************************************"""
    def SetMode(self,pMode):
        newAuto = (pMode == AUTOMATIC)
        if (newAuto and not self.inAuto):
            """we just went from manual to auto"""
            self.Initialize()
        self.inAuto = newAuto
    
    """ Initialize()****************************************************************
     *    does all the things that need to happen to ensure a bumpless transfer
     *  from manual to automatic mode.
     *****************************************************************************"""
    def Initialize(self):
        self.outputSum = self.outputSum*self.myOutput;
        self.lastInput = self.lastInput*self.myInput;
        if(self.outputSum > self.outMax):
            self.outputSum = self.outMax;
        elif (self.outputSum < self.outMin):
            self.outputSum = self.outMin;
    
    """ SetControllerDirection(...)*************************************************
     * The PID will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     *****************************************************************************"""
    def SetControllerDirection(self,pDirection):
        if(self.inAuto and pDirection != self.controllerDirection):
            self.kp = (0 - self.kp)
            self.ki = (0 - self.ki)
            self.kd = (0 - self.kd)
        self.controllerDirection = pDirection
    
    def setOutputSum(self,pOutputSum):
        if(pOutputSum > self.outMax): 
            self.outputSum= self.outMax;
        elif (pOutputSum < self.outMin): 
            self.outputSum= self.outMin;
        else:
            self.outputSum=pOutputSum
            
    """ Status Funcions*************************************************************
     * Just because you set the Kp=-1 doesn't mean it actually happened.  these
     * functions query the internal state of the PID.  they're here for display
     * purposes.  this are the functions the PID Front-end uses for example
     *****************************************************************************"""
    def GetKp(self): 
        return  self.dispKp
    def GetKi(self):
        return  self.dispKi
    def GetKd(self):
        return  self.dispKd
    def GetMode(self):
        if (self.inAuto):
            return AUTOMATIC
        else:
            return MANUAL
    def GetDirection(self):
        return self.controllerDirection
    
    def GetOutput(self):
        return self.myOutput
    
    def millis(self):
        if (self.startTime==0):
            self.startTime=int(round(time.time() * 1000))
        self.curentTime=int(round(time.time() * 1000))
            
        return self.curentTime-self.startTime

    def setInput(self,pInput):
        self.myInput = pInput;
        
    def setOutput(self,pOutput):
        self.myOutput = pOutput;
        
    def setSetpoint(self,pSetpoint):
        self.mySetpoint = pSetpoint;