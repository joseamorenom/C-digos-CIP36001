#!/usr/bin/env python3
"""
    The SampleRobot class is the base of a robot application that will
    automatically call your Autonomous and OperatorControl methods at
    the right time as controlled by the switches on the driver station
    or the field controls.
    
    WARNING: While it may look like a good choice to use for your code
    if you're inexperienced, don't. Unless you know what you are doing,
    complex code will be much more difficult under this system. Use
    IterativeRobot or Command-Based instead if you're new.
"""


import wpilib
from wpilib.drive import DifferentialDrive
'''
class PID():
    def __init__(self,ku,tu):
        
        self.P=0.6*ku
        self.I=(1.2*ku)/tu
        self.D=(3*ku*tu)/40
        self.error=0
        self.erroranterior=0
        self.integral=0
        self.derivada=0
        self.sp=0
        self.señal=0

    def setSetPoint(self,sp):
        self.sp=sp
    
    def getSetPoint(self):
        return self.sp
    
    def getSeñal(self,sensor):
        self.error=self.sp-sensor
        self.integral=self.integral+(self.error*0.01)
        self.derivada=(self.error-self.erroranterior)/0.01
        self.señal=self.error*self.P+self.I*self.integral+self.D*self.derivada
        self.erroranterior=self.error
        return self.señal
'''

class MyRobot(wpilib.SampleRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""
        
        self.lstick = wpilib.Joystick(0)
        self.encoder=wpilib.Encoder(0,1)
        self.encoderCascada = wpilib.Encoder(2,3)

        self.motordedo = wpilib.Spark(2)
        self.motorPelotas = wpilib.Spark(1)

        self.l_motor_del = wpilib.VictorSP(9)
        self.r_motor_del = wpilib.VictorSP(7)
        self.l_motor_tras = wpilib.VictorSP(5)
        self.r_motor_tras = wpilib.VictorSP(6)
        self.l_motor=wpilib.SpeedControllerGroup(self.l_motor_del,self.l_motor_tras)
        self.r_motor=wpilib.SpeedControllerGroup(self.r_motor_del,self.r_motor_tras)

        self.robot_drive = wpilib.drive.DifferentialDrive(self.l_motor, self.r_motor)

        self.servomotor=wpilib.Servo(8)
        self.valvula=wpilib.DoubleSolenoid(0,7)

        self.motorcascada1 = wpilib.Spark(2)
        self.motorcascada2 = wpilib.Spark(3)
        self.cascada = wpilib.SpeedControllerGroup(self.motorcascada1,self.motorcascada2)

    def disabled(self):
        """Called when the robot is disabled"""
        while self.isDisabled():
            wpilib.Timer.delay(0.01)

    def autonomous(self):
        """Called when autonomous mode is enabled"""

        while self.isAutonomous() and self.isEnabled():
            wpilib.Timer.delay(0.01)

    def operatorControl(self):
        """Called when operation control mode is enabled"""
        sp=0
        timer = wpilib.Timer()
        timer.start()
        aux=0
        aux2=0
        auxs = 0
        auxPelotas=0
        #ku=0.056
        #tu=0.04
        #ku=0.022
        #tu=0.0306
        P=0.6*ku
        #originalmente=0.6*0.01=0.006
        I=(1.2*ku)/tu 
        # #originalmente=(1.2*0.01)/0.127=0.0945
        D=(3*ku*tu)/40 
        #originalmente=(3*0.01*0.127)/40=0.00009525
        '''P=0.0004
        I=0.0945
        D=0.00012
        P=0.1'''
        #I=0.095
        #D=0.0000952
        erroranterior=0
        integral=0
        #self.pid1.setSetPoint(100)
        
        self.valvula.set(0)
        self.encoder.reset()
        self.encoderCascada.reset()
        
        while self.isOperatorControl() and self.isEnabled():
        
            self.robot_drive.arcadeDrive(-self.lstick.getY(),self.lstick.getX())
    
            #PELOTAS
            #Aquí se usa la Y de Pwm para que se tenga la misma señal en los Spark 0 y 1, 
            #aunque se debe de invertir la polaridad de uno de los motores según el movieminto del mecanismo
            if self.lstick.getRawButton(1)==True:
               self.motorPelotas.set(-1)
            else:
                self.motorPelotas.set(0)
            if self.lstick.getRawButton(2)==True:
                self.motorPelotas.set(0.5)
            else:
                self.motorPelotas.set(0)

            #NEUMÁTICA
            if self.lstick.getRawButtonPressed(3):
                if auxs==0:
                    print("sale")
                    self.servomotor.setAngle(128)
                    self.valvula.set(1)
                    #self.motordedo.set(0.5)
                    #self.motordedo2.set(0.5)
                    #self.encoder.get()
                    auxs=1
                elif auxs==1:
                    print("entra")
                    self.servomotor.setAngle(80)
                    self.valvula.set(2)
                    #self.encoder.get()
                    #print(self.encoder.get())
                    #self.motordedo.set(-0.5)
                    #self.motordedo2.set(-0.5)
                    auxs=0    
            
            #MEÑIQUE
            if self.lstick.getRawButtonPressed(4):
                if auxs==0:
                    self.servomotor.setAngle(126)
                    auxs=1
                elif auxs==1:
                    self.servomotor.setAngle(62)
                    auxs=0
            
            #DEDO
            print(self.encoder.get())
            if self.lstick.getRawButton(5)==True:
                if aux==0:
                    print("sale")
                    self.motordedo.set(0.5)
                elif aux==1:
                    self.motordedo.set(-0.5)
                    print("entra")
            if self.encoder.get()<=-18 and aux==0:
                       self.motordedo.set(0)
                       aux=1                  
            if self.encoder.get()>-2 and aux==1:
                        self.motordedo.set(0)
                        aux=0

            #ELEVADOR CASCADA
            if self.lstick.getRawButtonPressed(6):
                if aux2==0:
                    self.motorcascada1.set(-0.5)
                elif aux2==1:
                    self.motorcascada1.set(0.5)
            if self.encoderCascada.get()>=100 and aux2==0:
                self.motorcascada1.set(0)
                aux2 = 1                  
            if self.encoderCascada.get()<=-100 and aux2==1:
                self.motorcascada1.set(0)
                aux2 = 0
            print(self.encoderCascada.get())
    

            #ENCODER 1 MOTOR CIM
            if self.lstick.getRawButtonPressed(8):
                sp=157.5
            if self.lstick.getRawButtonPressed(9):
                sp=508
            if self.lstick.getRawButtonPressed(11):
                sp=853.75        
            
            
            #ENCODER TOUGHBOX
            sp=1000
           
            encoder=self.encoder.get()
            error=(sp-encoder)
            integral=integral+(error*0.01)
            derivada=(error-erroranterior)/0.01
            señal=error*P+I*integral+D*derivada
            if señal>1:
                señal=1
            if señal<-1:
                señal=-1
            self.cascada.set(señal)
            erroranterior=error
            print(encoder)

            wpilib.SmartDashboard.putNumber('sensor',encoder)
            wpilib.SmartDashboard.putNumber('set point', sp)

  
            wpilib.Timer.delay(0.01)


if __name__ == "__main__":
    wpilib.run(MyRobot)
