package frc.robot;

//import frc.robot.Map.EncoderPort;
import frc.robot.Map.SparkPort;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.Encoder;


public class Chassis {
    
    public Chassis() {
        
        //Instantiate Drive Train Motors
        try {
            leftMotor1  = new Spark(SparkPort.LEFT_DRIVE1);
            rightMotor3  = new Spark(SparkPort.RIGHT_DRIVE3);
            leftMotor2 = new Spark(SparkPort.LEFT_DRIVE2);
            rightMotor4 = new Spark(SparkPort.RIGHT_DRIVE4);
            leftDrive   = new DifferentialDrive(leftMotor1, leftMotor2);
            rightDrive   = new DifferentialDrive(rightMotor3, rightMotor4);
        } catch(Exception ex) {
            DriverStation.reportError("Could not instantiate Drive Train Motors\n", false);
        }

     /*   try {
            encoder = new Encoder(EncoderPort.One_A, EncoderPort.One_B, true);
        } catch(Exception e) {
            DriverStation.reportError("Could not instantiate Encoder: One_A\n", false);
        }
*/


    }
    
    public void stop() {
        leftDrive.arcadeDrive(0.0, 0.0);
        rightDrive.arcadeDrive(0.0, 0.0);
        
    }
    
    public void periodic(Joystick stick) {
        if(stick == null) {
            DriverStation.reportError("No Joystick, cannot run Chassis periodic\n", false);
            return;
        }
        
        leftDrive.arcadeDrive(stick.getY(), -stick.getX());
        rightDrive.arcadeDrive(stick.getY(), -stick.getX());

       // if (encoder.getStopped() != true){
       // System.out.print(encoder.getDistance());
        //System.out.print(encoder.getDirection());
    }
//}
    private DifferentialDrive leftDrive, rightDrive;
    private Spark            leftMotor1 , leftMotor2, rightMotor3, rightMotor4 ; 

    //private Encoder encoder;
}
