'''
Created on 21 janv. 2018

@author: OLIVIERCousinier
'''
import sys, traceback
import socket
from urllib.request import urlopen

class RobotCom(object):
    def __init__(self,name,host,port,path=''):
        self.name = name
        self.host = host
        self.port = port
        self.path = path
        self.sock=None
    
    def open(self):
        self.sock=True
        return
    
    def send(self,pMessage):
        if self.sock is None:
            self.open()
        return
    
    def sendWithResponse(self,pMessage):
        if self.sock is None:
            self.open()
        return
    
    def recieve(self):
        if self.sock is None:
            self.open()
        return
    
    def close(self):
        self.sock=None
        return
    
    def quit(self):
        self.close()
    
class RobotComSerial(RobotCom):
    def __init__(self,name, host,port):
        RobotCom.__init__(self, name, host, port)
        self.sock=None
        self.timeout=10 #connection time out in s
 
    def sendWithResponse(self,pMessage):
        if self.sock is None:
            self.open()
            
        #print("==============================================\n",pMessage)
        self.sock.sendall(pMessage)
        packet =""
        EndOfData=False;
        data="";
        
        while not EndOfData:
            try:
                packet  = self.sock.recv(4096)
                #print('packet=',packet.decode("UTF-8"), end='')
                data+=packet.decode("UTF-8")
                if (data.index("[endOfData]")>0):
                    #data+="[found endOfData]\n"
                    EndOfData=True;                   
            except Exception as ex:
                #template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                #message = template.format(type(ex).__name__, ex.args)
                #print (message)
                #print (traceback.format_exc())

                if (type(ex).__name__!="ValueError"):
                    #data+="[EndOfData not found]\n"
                    break
        #print('\n------\nSent data=',pMessage,"\nRecieved Data:",str(data), end='')
        return data
        
        
        """
        data=""
        EndOfData=False;
        while not EndOfData:
            try:
                data = self.sock.recv(4096)
                print (data.decode("UTF-8"), end='')
                if data.index("\n")>0:
                    print ("found EOL")
                    EndOfData=True;
            except:
                print("No response")
                break
        """
        
    def recieve(self):
        if self.sock is None:
            self.open()
        EndOfData=False;
        data="";
        
        while not EndOfData:
            try:
                packet  = self.sock.recv(4096)
                #print('packet=',packet.decode("UTF-8"), end='')
                data+=packet.decode("UTF-8")
                if (data.index("[endOfData]")>0):
                    data+="[found endOfData]\n"
                    EndOfData=True;                   
            except Exception as ex:
                #template = "An exception of type {0} occurred. Arguments:\n{1!r}"
                #message = template.format(type(ex).__name__, ex.args)
                #print (message)
                #print (traceback.format_exc())

                if (type(ex).__name__!="ValueError"):
                    data+="[EndOfData not found]\n"
                    break
        print('\n------\nRecieved Data:',str(data), end='')
        return data
        
    def quit(self):
        self.close()
        
    def open(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(self.timeout)
            self.sock.connect((self.host, self.port))
        except:
            print("Could not open communication")
            self.sock=None
            raise
    
    def send(self,pMessage):
        if self.sock is None:
            self.open()
            
        if pMessage is None:
            return
            
        self.sock.sendall(pMessage)
        """
        packet =""
        EndOfData=False;
        data="";
        
        while not EndOfData:
            try:
                packet  = self.sock.recv(4096)
                #print('packet=',packet.decode("UTF-8"), end='')
                data+=packet.decode("UTF-8")
                #print("Recieve Data:",data)
                if data.index("\n")>0:
                    data+="[found EOL]"
                    EndOfData=True;
            except:
                data+=b'[EndOfData not found]'
                break
        print('Sent data=',pMessage,"\nRecieved Data:",data, end='')
        """
        
    
    def close(self):
        if self.sock is not None:
            self.sock.close()
            
class RobotComUrl(RobotCom):
    def __init__(self,pName, pType='http', pPath='/', pHost='localhost', pPort='8080'):
        RobotCom.__init__(self, pName, pHost, pPort,pPath)
        self.type=pType
        if self.type=='file':
            self.url =pPath
        else:
            self.url ='http://'+self.host+':'+str(self.port)+pPath
        self.sock=None
    
    def open(self):
        self.sock= urlopen(self.url) 
        return
    
    def send(self,pMessage):
        if self.sock is None:
            self.open();
        return
    
    def receive(self):
        #Force for IP Webcam
        if (self.sock is None or self.path=='/shot.jpg'):
            self.open();
        return  self.sock.read()
    
    def receiveLine(self):
        if self.sock is None:
            self.open();
        
        return  self.sock.read()
    
    def close(self):
        return

