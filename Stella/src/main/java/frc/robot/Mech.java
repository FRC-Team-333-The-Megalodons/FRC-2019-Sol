package frc.robot;

//import frc.robot.Map.TalonPort;
import frc.robot.Map.SparkPort;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;

public class Mech {
    
    public static final double MOTOR_POWER = 0.4;

    public Mech() {
        
     /*   //Instantiate Test Motor w/ Encoder
        try {
            talon = new TalonSRX(TalonPort.TEST_MOTOR);
        } catch(Exception ex) {
            DriverStation.reportError("Could not instatiate test motor\n", false);
        }
     */   
  /*      //Instantiate Test Motor
        try {
            motor = new Spark(SparkPort.MOTOR_A);
        }catch (Exception ex) {
            DriverStation.reportError("Coudn't instantitate Motor_A",false);
        }
     */   
    }
    
    public void periodic(Joystick stick) {
        if(stick == null) {
            DriverStation.reportError("No Joystick, cannot run Mech periodic\n", false);
            return;
        }
        
       /* if (stick.getRawButton(2)) {
            motor.set(MOTOR_POWER);
        } else if (stick.getTrigger()) {
            talon.set(ControlMode.PercentOutput, MOTOR_POWER); //how to give power to motors using the Talon SRX
        } else {
            talon.set(ControlMode.PercentOutput, 0.0);
            motor.set(0);
        }
      */  
        //testMotor.getMotionProfileStatus(); 
    }
 
    //private TalonSRX talon;
    private Spark   motor;
        
    }
    

