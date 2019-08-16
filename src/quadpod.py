# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 18:41:55 2019

@author: miguel
"""
import numpy
from evdev import InputDevice, categorize, ecodes
import serial
import time
from simple_pid import PID
from select import select

####################################################################################    
##############################______MATEMATICAS_______##############################
#################################################################################### 
def Rx(pitch):
    """ Rotation matrix arround x (pitch)
    """
    pitch = numpy.radians(pitch)
    return numpy.matrix([[1,               0,                0],
                         [0, numpy.cos(pitch), -numpy.sin(pitch)],
                         [0, numpy.sin(pitch),  numpy.cos(pitch)]])

def Ry(roll):
    """ Rotation matrix arround y (roll)
    """
    roll = numpy.radians(roll)
    return numpy.matrix([[ numpy.cos(roll), 0, numpy.sin(roll)],
                         [              0, 1,             0],
                         [-numpy.sin(roll), 0, numpy.cos(roll)]])

def Rz(yaw):
    """ Rotation matrix arround z (yaw)
    """
    yaw = numpy.radians(yaw)
    return numpy.matrix([[numpy.cos(yaw), -numpy.sin(yaw), 0],
                         [numpy.sin(yaw),  numpy.cos(yaw), 0],
                         [             0,               0, 1]])

def matrixToAxisAngle(R):
    """ Rotation matrix to axis/angle
    """
    d = 1 / numpy.sqrt(numpy.power(R[2, 1] - R[1, 2], 2) + numpy.power(R[0, 2] - R[2, 0], 2) + numpy.power(R[1, 0] - R[0, 1], 2))
    x = (R[2, 1] - R[1, 2]) * d
    y = (R[0, 2] - R[2, 0]) * d
    z = (R[1, 0] - R[0, 1]) * d
    angle = numpy.arccos((R[0, 0] + R[1, 1] + R[2, 2] - 1) / 2)

    return     numpy.array([x, y, z]), angle

def rotPointtoAngleaxis(point,axis,angle):
    """Point, axis and angle to vector
    """ 
    A=axis[0]**2+axis[1]**2+axis[2]**2
    x=(axis[0]*(axis[0]*point[0]+axis[1]*point[1]+axis[2]*point[2])*(1-numpy.cos(angle))+A*point[0]*numpy.cos(angle)+numpy.sqrt(A)*(-axis[2]*point[1]+axis[1]*point[2])*numpy.sin(angle))/A
    y=(axis[1]*(axis[0]*point[0]+axis[1]*point[1]+axis[2]*point[2])*(1-numpy.cos(angle))+A*point[1]*numpy.cos(angle)+numpy.sqrt(A)*(axis[2]*point[0]-axis[0]*point[2])*numpy.sin(angle))/A
    z=(axis[2]*(axis[0]*point[0]+axis[1]*point[1]+axis[2]*point[2])*(1-numpy.cos(angle))+A*point[2]*numpy.cos(angle)+numpy.sqrt(A)*(-axis[1]*point[0]+axis[0]*point[1])*numpy.sin(angle))/A
    
    return     numpy.array([x,y,z])
    
    
def checkdomain(D):
    if D > 1 or D < -1:
        print("____OUT OF DOMAIN____")
        if D > 1: 
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D
        
        """
        INVERSE KINEMATICS
        """
def solveIK_R(x,y,z):
    coxa=3.6
    femur=6.1
    tibia=9.4
    D = (x**2+y**2-coxa**2+z**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(-y,x)-numpy.arctan2(numpy.sqrt(x**2+y**2-coxa**2),-coxa) - numpy.pi/2
    alpha = numpy.arctan2(z,numpy.sqrt(x**2+y**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    return tetta, alpha, gamma
    
def solveIK_L(x,y,z):
    coxa=3.6
    femur=6.1
    tibia=9.4
    D = (x**2+y**2-coxa**2+z**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    D = checkdomain(D)
    gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    tetta = -numpy.arctan2(-y,x)-numpy.arctan2(numpy.sqrt(x**2+y**2-coxa**2),-coxa) - numpy.pi/2 
    alpha = numpy.arctan2(z,numpy.sqrt(x**2+y**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    return tetta, alpha, gamma


#TRYING TO CALCULATE POSITION WITH ACCELEROMETER (TOO MUCH ERROR)
#class calculate():       ============================================NO CREO QUE FUNCIONES
#    v0=numpy.array([0 , 0])                                            DEMASIADO ERROR
#    s0=numpy.array([0 , 0])
#    def Pos(self , d_t , Xacc , Yacc):
#        if (Xacc < 0.3 and Xacc > -0.3):
#            Xacc=0
#        if (Yacc < 0.3 and Yacc > -0.3):
#            Yacc=0
#        d_t=d_t/1000
#        acc=numpy.array([Xacc , Yacc])
#        self.v0= acc*d_t
#        
#        s=self.s0 + self.v0*d_t + 0.5*acc*d_t**2
#        self.s0=s
#        return s
    
    
    
    
    
####################################################################################    
####################______FUNCTIONS FOR UP/DOWN STEP_______######################### POR TERMINAR
#################################################################################### 
def upstep(x0 , t , betta):   #(esto es la mitad de la longitud del paso, tiempo, angulo)
    y0=x0**2
    betta=(2*numpy.pi*betta)/360
#    t = numpy.arange(-x0, x0, 0.1)
    step=[]
    #cylindrical coordinates
    x=(x0+t)*numpy.cos(betta)
    z=(x0+t)*numpy.sin(betta)
    y=-y0+t**2
    step=numpy.array([x,y,z])
    return step

def downstep(x0 , t , betta):
    betta=(2*numpy.pi*betta)/360
#    t = numpy.arange(-x0, x0, 0.1)
    step=[]
    x=(x0-t)*numpy.cos(betta)
    z=(x0-t)*numpy.sin(betta)
    y=0
    step=numpy.array([x,y,z])
    return step



####################################################################################    
################______CONVERTIONS ANGLE --> PULSE_______##########################
#################################################################################### 
class convert():
    #do all convertions at ones
    offsetCoxa=7*numpy.pi/4 
    def servos(self , tetta0 , tetta1 , tetta2 , tetta3 , alpha0, alpha1 , alpha2 , alpha3 , gamma0 , gamma1 , gamma2 , gamma3):
        pulse1=self.angtoPulse1(-alpha1)
        pulse2=self.angtoPulse2(gamma1)
        pulse3=self.angtoPulse3(-alpha0)
        pulse4=self.angtoPulse4(gamma0)
        pulse5=self.angtoPulse5(-alpha3)
        pulse6=self.angtoPulse6(gamma3)
        pulse7=self.angtoPulse7(-alpha2)
        pulse8=self.angtoPulse8(gamma2)
        pulseFR=self.angtoPulseFR(tetta0+self.offsetCoxa)
        pulseFL=self.angtoPulseFL(tetta1+self.offsetCoxa)
        pulseBR=self.angtoPulseBR(tetta2+self.offsetCoxa)
        pulseBL=self.angtoPulseBL(tetta3+self.offsetCoxa)
        allPulses=numpy.array([pulse1 , pulse2 , pulse3 , pulse4 , pulse5 , pulse6 , pulse7 , pulse8 , pulseFR , pulseFL , pulseBR , pulseBL])        
        
        return allPulses
        
        
        #FUNCTIONS FOR CONVERT ANGLES TO SERVO PULSE
    def angtoPulse1(self , ang):
        y1=2200
        y2=780
        x1=0
        x2=-157*numpy.pi/180
        pulse1=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse1)
        
    def angtoPulse2(self , ang):
        y1=1860
        y2=670
        x1=0
        x2=-150*numpy.pi/180
        pulse2=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse2)
        
    def angtoPulse3(self , ang):
        y1=830
        y2=2175-40
        x1=0
        x2=157*numpy.pi/180
        pulse3=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse3)
        
    def angtoPulse4(self , ang):
        y1=870
        y2=2080
        x1=0
        x2=150*numpy.pi/180
        pulse4=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse4)
        
    def angtoPulse5(self , ang):
        y1=1925
        y2=540
        x1=0
        x2=-157*numpy.pi/180
        pulse5=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse5)
        
    def angtoPulse6(self , ang):
        y1=1940
        y2=610
        x1=0
        x2=-150*numpy.pi/180
        pulse6=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse6)
        
    def angtoPulse7(self , ang):
        y1=935
        y2=2340-20
        x1=0
        x2=157*numpy.pi/180
        pulse7=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse7)
        
    def angtoPulse8(self , ang):
        y1=970
        y2=2260
        x1=0
        x2=150*numpy.pi/180
        pulse8=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulse8)
        
    def angtoPulseFR(self , ang):
        y1=975
        y2=1410
        x1=0
        x2=-43*numpy.pi/180
        pulseFR=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulseFR)
        
    def angtoPulseFL(self , ang):
        y1=1686
        y2=1305
        x1=0
        x2=-43*numpy.pi/180
        pulseFL=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulseFL)
        
    def angtoPulseBR(self , ang):
        y1=1090
        y2=1620
        x1=0
        x2=-43*numpy.pi/180
        pulseBR=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulseBR)
        
    def angtoPulseBL(self , ang):    
        y1=1855
        y2=1420
        x1=0
        x2=-43*numpy.pi/180
        pulseBL=y1+((y2-y1)/(x2-x1))*(ang-x1)
        return int(pulseBL)

    
    
####################################################################################    
#############################______BODY CLASS_______################################
####################################################################################  
class body():
    
    #PROPIEDADES DE LA CLASE#
    dx=0.
    dy=0.
    dz=0.
    
    d_tetta=[0.,0.,0.,0.]
    d_alpha=[0.,0.,0.,0.]
    d_gamma=[0.,0.,0.,0.]
    d_yaw=0.
    d_roll=0.
    d_pitch=0.
    
    """ROBOT DIMENTIONS"""
    #cuerpo
    length=22.
    height=3.
    width=10.
    #piernas
    coxa_length=3.6
    femur_length=6.1
    tibia_length=9.4
    
    CM=[dx,dy,dz]
    
    body_direction=numpy.array([1,0,0])
    body_size=numpy.array([length,height,width])
          
    coxa_posFR=numpy.array([length/2 , 0 , width/2])
    coxa_posFL=numpy.array([length/2 , 0. , -width/2])
    coxa_posBR=numpy.array([-length/2 , 0. , width/2])
    coxa_posBL=numpy.array([-length/2 , 0. , -width/2])
    coxa_axis=numpy.array([-coxa_length , 0. , 0.])
    cadera_axis=numpy.array([0. , 0. , coxa_length])
        
    femur_posL=numpy.array([0. , femur_length/2 , coxa_length/2])
    femur_posR=numpy.array([0. , femur_length/2 , -coxa_length/2])
    femur_size=numpy.array([1. , femur_length , coxa_length])
    rodilla_pos=numpy.array([0. , -femur_length , 0.])
    rodilla_axis=numpy.array([0. , 0. , coxa_length])
    
    tibia_posL=numpy.array([0. , tibia_length/2 , coxa_length/2])
    tibia_posR=numpy.array([0. , tibia_length/2 , -coxa_length/2])
    tibia_size=numpy.array([1. , tibia_length , coxa_length/2])
    
    leg_posR=numpy.array([0. , -8. , -1.]) ###posicion del SR pie en funcion del SR cuerpo
    leg_posL=numpy.array([0. , -8. , --1.]) #¿POSICION INICIAL?
      
####################################################################################    
######################______CALCULATES MOTOR ANGLES_______##########################
#################################################################################### 
    def stability(self , _yaw , _roll , _pitch , _x , _y , _z , step_fr , step_fl , step_br , step_bl):     #calcula angulos 
        """rotaciones del cuerpo""" 
        """posicion inicial (-15.4,-2,0)"""
#        self._yaw=yaw
#        self._roll=roll
#        self._pitch=pitch      
        feetcte1=numpy.array([0.,0.,0.])
        feetcte2=numpy.array([0.,0.,0.])
        feetcte3=numpy.array([0.,0.,0.])
        feetcte4=numpy.array([0.,0.,0.])
        translation1=numpy.array([0.,0.,0.])
        translation2=numpy.array([0.,0.,0.])
        translation3=numpy.array([0.,0.,0.])
        translation4=numpy.array([0.,0.,0.])       
        coxatofeet1=numpy.array([0.,0.,0.])
        coxatofeet2=numpy.array([0.,0.,0.])
        coxatofeet3=numpy.array([0.,0.,0.])
        coxatofeet4=numpy.array([0.,0.,0.])
        _tetta=numpy.array([0.,0.,0.,0.])
        _alpha=numpy.array([0.,0.,0.,0.])
        _gamma=numpy.array([0.,0.,0.,0.])
        
        #vector from CM to the feet that must be constant (for static balance)
        feetcte1=self.coxa_posFR + self.leg_posR
        feetcte2=self.coxa_posFL + self.leg_posL
        feetcte3=self.coxa_posBR + self.leg_posR
        feetcte4=self.coxa_posBL + self.leg_posL
        
        #vector from CM to coxa in order to calculate the rotation
        rotcoxa1=self.coxa_posFR
        rotcoxa2=self.coxa_posFL
        rotcoxa3=self.coxa_posBR
        rotcoxa4=self.coxa_posBL
        
        #vector to locate the CM
        translation1=numpy.array([_x,_y,_z])
        translation2=numpy.array([-_x,_y,_z])
        translation3=numpy.array([_x,_y,_z])
        translation4=numpy.array([-_x,_y,_z])
        
        #the matrix for the rotations
        Rr=Ry(-_roll) * Rz(_yaw) * Rx(-_pitch)    #deshace el giro de SR de las IK para mantener el pie en pos
        Rl=Ry(_roll) * Rz(-_yaw) * Rx(_pitch)
        if _yaw or _roll or _pitch != 0.:
            
             axis_r, angle_r = matrixToAxisAngle(Rr)    
             axis_l, angle_l = matrixToAxisAngle(Rl)             
             
             #then rotate the CM to coxa vector
             rotcoxa1=rotPointtoAngleaxis(rotcoxa1,axis_r,-angle_r) 
             rotcoxa2=rotPointtoAngleaxis(rotcoxa2,axis_l,angle_l)
             rotcoxa3=rotPointtoAngleaxis(rotcoxa3,axis_r,-angle_r)
             rotcoxa4=rotPointtoAngleaxis(rotcoxa4,axis_l,angle_l)
             
#        print('pos inicial pie')
#        print(feetcte1)
#        print(rotcoxa1)
        
        #now calculate the final vector for the IK, whith is the vector from coxa to the feet point (wih rotations done)
        coxatofeet1=rotcoxa1+translation1-feetcte1+step_fr
        coxatofeet2=rotcoxa2+translation2-feetcte2+step_fl
        coxatofeet3=rotcoxa3+translation3-feetcte3+step_br
        coxatofeet4=rotcoxa4+translation4-feetcte4+step_bl

        """"" y=-z ; z=x ; x=y """""
#        print(coxatofeet1)
#        print(coxatofeet2)
#        print(coxatofeet3)
#        print(coxatofeet4)
        
        tetta1, alpha1, gamma1=solveIK_R(coxatofeet1[1],-coxatofeet1[2],coxatofeet1[0])
        
        tetta2, alpha2, gamma2=solveIK_L(coxatofeet2[1],coxatofeet2[2],coxatofeet2[0])
        
        tetta3, alpha3, gamma3=solveIK_R(coxatofeet3[1],-coxatofeet3[2],coxatofeet3[0])

        tetta4, alpha4, gamma4=solveIK_L(coxatofeet4[1],coxatofeet4[2],coxatofeet4[0])
            ####TIBIA######## +,-,+,-
        _gamma=[gamma1,gamma2, gamma3, gamma4]
            ####FEMUR####### +,-,+,-
        _alpha=[alpha1,alpha2, alpha3, alpha4]
            ####COXA####
        _tetta=[tetta1-numpy.pi,tetta2-numpy.pi,tetta3-numpy.pi,tetta4-numpy.pi]
#        print(numpy.degrees(_tetta))
#        print(numpy.degrees(_alpha))
#        print(numpy.degrees(_gamma))
        
        return _tetta , _alpha , _gamma



####################################################################################    
######################______JOYSTICK FUNCTION_______################################
####################################################################################  
class Joystick():
    gamepad = InputDevice('/dev/input/by-id/usb-SAITEK_P880-event-joystick')
    d_x=0.
    d_y=0.
    d_z=0.
    d_rz=0.
    d_pitch=0.
    d_yaw=0.
    d_roll=0.
    def Read(self):
        r,w,x = select([self.gamepad.fd], [], [], 0.1)
        d_y=0.
        if r:
            for event in self.gamepad.read():
    #            print(event)
                if event.type == ecodes.EV_KEY:
                    if event.value == 1:
                        if event.code == 288:
                            print(d_y)
                        if event.code == 289:
                            print(d_y)
                    else:
                        print("boton soltado")
                ########################################  for my own joystick
                #      ^           #     ^             #
                #    ABS_Y         #    ABS_Z          #
                #  ←─────→ ABS_X #  ←─────→ ABS_RZ  #
                #     ↓           #     ↓            #  
                #######################################
                elif event.type == ecodes.EV_ABS:
                    absevent = categorize(event)
                    if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X":  
                        self.d_x=(absevent.event.value-127)/40.
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y":
                        self.d_y=(absevent.event.value-127)/40.
                        
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z":
                        self.d_z=numpy.abs(absevent.event.value-127)/40.
#                        print(self.d_z)
                    elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ":
                        self.d_rz=(absevent.event.value-127)/45.
                        
        return self.d_x , self.d_y , self.d_rz , self.d_z

####################################################################################    
###################______ARDUINO SERIAL CLASS_______################################
####################################################################################    
class ArduinoSerial():  #clase para la comunicacion con arduino mediante serial, se inicia diciendo puerto y velocidad
                        #y en la funcion crea un string con el formato para la lectura.
    arduino = serial.Serial('/dev/ttyACM0', 115200, timeout = 1)
    # Nota: provocamos un reseteo manual de la placa para leer desde
    # el principio
    arduino.setDTR(False)
    time.sleep(1)
    arduino.flushInput()
    arduino.setDTR(True)
    time.sleep(2)
        
    def serialSend(self, pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulseFR, pulseFL, pulseBR, pulseBL):  
        comando = "<{0}#{1}#{2}#{3}#{4}#{5}#{6}#{7}#{8}#{9}#{10}#{11}>" #Input
        command=comando.format(pulse1, pulse2, pulse3, pulse4, pulse5, pulse6, pulse7, pulse8, pulseFR, pulseFL, pulseBR, pulseBL)
        self.arduino.write(command) #Mandar un comando hacia Arduino
#        serialin=self.arduino.readline()
#        print(command)      
    

    def serialRecive(self):
        try:
            
            startMarker = 60
            endMarker = 62
      
            getSerialValue = ""
            x = "z" # any value that is not an end- or startMarker
            byteCount = -1 # to allow for the fact that the last increment will be one too many
                  
                  # wait for the start character
            while  ord(x) != startMarker: 
                x = self.arduino.read()
                  
                  # save data until the end marker is found
            while ord(x) != endMarker:
                if ord(x) != startMarker:
                    getSerialValue = getSerialValue + x 
                    byteCount += 1
                x = self.arduino.read()
                  
#            print(getSerialValue)          
            time , pitch, yaw  = numpy.fromstring(getSerialValue.decode('ascii', errors='replace'), sep = '#' )
    #            print(time, pitch , yaw , roll)
#            if time or pitch or yaw or roll == None:
#                print("none") 
                
        except ValueError:
            print("______lee siguiente linea_____")
            startMarker = 60
            endMarker = 62
      
            getSerialValue = ""
            x = "z" # any value that is not an end- or startMarker
            byteCount = -1 # to allow for the fact that the last increment will be one too many
                  
                  # wait for the start character
            while  ord(x) != startMarker: 
                x = self.arduino.read()
                  
                  # save data until the end marker is found
            while ord(x) != endMarker:
                if ord(x) != startMarker:
                    getSerialValue = getSerialValue + x 
                    byteCount += 1
                x = self.arduino.read()
                  
#            print(getSerialValue)          
            time , pitch, yaw = numpy.fromstring(getSerialValue.decode('ascii', errors='replace'), sep = '#' )
        
        self.arduino.flushInput()    
        return time , -pitch, -yaw
            
    def close(self):
        self.arduino.close()
        

############################################################################
#######################______PID control_____###############################
############################################################################       

class PIDcontrol():
    yawPID = PID(0.1, 2, 0.004, setpoint=2.)
    pitchPID = PID(0.1, 2, 0.004, setpoint=2.)
    
    xPID = PID(0.05, 1, 0.001, setpoint=4)
    zPID = PID(0.05, 1, 0.001, setpoint=4)
        
    def rotacion(self , yaw, pitch):

        yawControl=self.yawPID(yaw)
        pitchControl=self.pitchPID(pitch)
            
        return yawControl , pitchControl
            
    def translacion(self , yaw, pitch):

        yawControl=self.xPID(yaw)
        pitchControl=self.zPID(pitch)
            
        return yawControl , pitchControl
    



############################################################################
#######################______main()_______##################################
############################################################################            
def main():
    
    #initialize all the classes
    Body=body() 
    arduino=ArduinoSerial()
    PID=PIDcontrol()    
    joystick=Joystick()
    move=convert() 

    
    #define the initial position and orientation
    d_x=-1.5    
    d_y=0.
    d_z=0.
    d_pitch=0.
    d_roll=0.
    d_yaw=0.
    
    #define the initial position of the feets
    h=10
    sep=3.59    
    Body.leg_posR=numpy.array([0. , -h , sep])  
    Body.leg_posL=numpy.array([0. , -h , -sep])
    
    direc=0     ##angle(º) of the cilindrical coordenates of the step
    L=2.2       ##radious of the cilindrical coordenates, the lenght of the step
    tu = numpy.arange(-L, L, 0.2)   #for every upstep the downstep should travel 1/3 of the total step lenght
    td = numpy.arange(-L, L, 0.06666)
    
    offsetCoxa=7*numpy.pi/4   
    
    
    print("_________INITIAL ROUTINE DONE__________")
    
    
    
    ##################################################################STATIC TEST
    while(True):

        loopTime , pitch , yaw = arduino.serialRecive() #ORIENTACION REAL
        d_y , d_x , d_z , nada = joystick.Read()
        
#        print(d_x,d_y,d_z)
#        d_yaw, d_pitch = PID.rotacion(yaw , pitch)  #se le suma el angulo que tenga el mando

#        
#        print(yaw , pitch)
##        
#        d_x= -1 + yaw/compensateXZ
#        d_z= -pitch/compensateXZ
        
        tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z,(0,0,0),(0,0,0),(0,-4,0),(0,0,0))
        
#        tetta=numpy.array([-2*numpy.pi,-2*numpy.pi,-2*numpy.pi,-2*numpy.pi])      
#        alpha=numpy.array([-numpy.deg2rad(90),numpy.deg2rad(90),-numpy.deg2rad(90),numpy.deg2rad(90)]) 
#        gamma=numpy.array([numpy.deg2rad(90),-numpy.deg2rad(90),numpy.deg2rad(90),-numpy.deg2rad(90)]) 
        
        pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
        arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
        time.sleep(0.01)    #YA QUE ESTE BUCLE DEBE DURAR, SI NO LO MISMO, ALGO MAS
    arduino.close()
#    
#    ##################################################################DINAMIC TEST
    while(True):
        for i in range(len(tu)):

            loopTime , pitch , yaw = arduino.serialRecive() #ORIENTACION REAL
#            d_yaw, d_pitch = PID.rotacion(yaw , pitch)      
            d_x= -3 + yaw/compensateXZ
            d_z= -pitch/compensateXZ
            print(yaw , pitch)
            tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z,downstep(L,tu[i],0),upstep(L,tu[i],180) ,upstep(L,tu[i],0) ,downstep(L,tu[i],180))
            pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
            arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
            time.sleep(0.03)
            
        for i in range(len(tu)):

            loopTime , pitch , yaw = arduino.serialRecive() #ORIENTACION REAL
#            d_yaw, d_pitch = PID.rotacion(yaw , pitch)
            d_x= -3 + yaw/compensateXZ
            d_z= -pitch/compensateXZ
            print(yaw , pitch)

            tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z,upstep(L,tu[i],0),downstep(L,tu[i],180) ,downstep(L,tu[i],0) ,upstep(L,tu[i],180))
            pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
            arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
            time.sleep(0.03)    
#    
#    
   ######################################################################## CREEP GAIT LOOP
    while(True):
        
            
        for i in range(len(tu)):
            print(len(tu))
            loopTime , pitch , yaw = arduino.serialRecive() #ORIENTACION REAL
            d_yaw, d_pitch = PID.rotacion(yaw , pitch)
#            angle_h , angle_v , nada , L  = joystick.Read()


            d_z=-numpy.log(i+1)
            d_x=-numpy.log(i+1) - 3.
            tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z , downstep(L,td[i+40],0),downstep(L,td[i],180) ,upstep(L,tu[i],0) ,downstep(L,td[i+20],180))
            pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
            arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
            time.sleep(0.04)
            
        for i in range(len(tu)):

            loopTime , pitch, yaw = arduino.serialRecive() #ORIENTACION REAL
            d_yaw, d_pitch = PID.rotacion(yaw , pitch)      
#            angle_h , angle_v , nada , L = joystick.Read()


            d_z=-numpy.log(len(tu))
            d_x=-numpy.log(len(tu))+numpy.log(i+1) - 3.
            tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z , upstep(L,tu[i],0),downstep(L,td[i+20],180) ,downstep(L,td[i],0) ,downstep(L,td[i+40],180))
            pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
            arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
            time.sleep(0.04)
            
        for i in range(len(tu)):

            loopTime , pitch , yaw = arduino.serialRecive() #ORIENTACION REAL
            d_yaw, d_pitch = PID.rotacion(yaw , pitch)
#            angle_h , angle_v , nada , L = joystick.Read()


            d_z=numpy.log(i+1)
            d_x=-numpy.log(i+1) - 3.
            tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z , downstep(L,td[i],0),downstep(L,td[i+40],180) ,downstep(L,td[i+20],0) ,upstep(L,tu[i],180))
            pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
            arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
            time.sleep(0.04)    
            
        for i in range(len(tu)):

            loopTime , pitch , yaw = arduino.serialRecive() #ORIENTACION REAL
            d_yaw, d_pitch = PID.rotacion(yaw , pitch)
#            angle_h , angle_v , nada , L = joystick.Read()


            d_z=numpy.log(len(tu))
            d_x=-numpy.log(len(tu))+numpy.log(i+1) - 3.
            tetta,alpha,gamma=Body.stability(d_yaw,d_roll,d_pitch,d_x,d_y,d_z , downstep(L,td[i+20],0),upstep(L,tu[i],180) ,downstep(L,td[i+40],0) ,downstep(L,td[i],180))
            pulse = move.servos(tetta[0] , tetta[1] , tetta[2] , tetta[3] , alpha[0] , alpha[1] , alpha[2] , alpha[3] , gamma[0] , gamma[1] , gamma[2] , gamma[3])
            arduino.serialSend(pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6], pulse[7], pulse[8], pulse[9], pulse[10], pulse[11])
            time.sleep(0.04)
            
        
    arduino.close()
#            



        
        
if __name__== "__main__":
    main()

    
