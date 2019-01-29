package frc.robot;

//import frc.robot.Map.EncoderPort;
import frc.robot.RobotMap.SparkPort;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.Encoder;


public class RobotChassis {
    
    public RobotChassis() {
        
        //Instantiate Drive Train Motors
      /*  try {
            SpeedController leftDrive =  new MultiSpeedController(new Spark(SparkPort.LEFT_DRIVE1),
                                                                  new Spark(SparkPort.LEFT_DRIVE2));

            SpeedController rightDrive = new MultiSpeedController(new Spark(SparkPort.RIGHT_DRIVE3),
                                                                  new Spark(SparkPort.RIGHT_DRIVE4));

            m_rawDifferentialDrive = new DifferentialDrive(leftDrive, rightDrive);
        } catch(Exception ex) {
            DriverStation.reportError("Could not instantiate Drive Train Motors\n", false);
        }
*/
        /* 
        try {
            encoder = new Encoder(EncoderPort.One_A, EncoderPort.One_B, true);
        } catch(Exception e) {
            DriverStation.reportError("Could not instantiate Encoder: One_A\n", false);
        }
        */
    }
    
    public void stop() {
        m_rawDifferentialDrive.arcadeDrive(0.0, 0.0);        
    }
    
    public void periodic(Joystick stick) {
        if(stick == null) {
            DriverStation.reportError("No Joystick, cannot run Chassis periodic\n", false);
            return;
        }

      //  m_rawDifferentialDrive.arcadeDrive(stick.getY(), -stick.getX());
    }
    private DifferentialDrive m_rawDifferentialDrive;
    
    //private TeleopTransDrive m_drive;
    //private Encoder encoder;
}
