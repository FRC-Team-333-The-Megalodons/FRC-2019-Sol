package frc.robot;

//import frc.robot.Map.EncoderPort;
import frc.robot.RobotMap.SparkPort;
import frc.robot.RobotMap.SolenoidPort;
import edu.wpi.first.wpilibj.Compressor;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.Encoder;


public class RobotChassis {
    
    public RobotChassis() {

                /* Instantiate the compressor */
                try {
                    m_compressor = new Compressor(RobotMap.CompressorPort.MAIN_COMPRESSOR);
                } catch (Exception ex) {
                    DriverStation.reportError("Could not start Compressor\n", false);
                }
        
        //Instantiate Drive Train Motors
        try {
            m_transmission = new Solenoidal(SolenoidPort.DRIVE_TRANS_LOW, SolenoidPort.DRIVE_TRANS_HIGH);
            SpeedController leftDrive =  new MultiSpeedController(new Spark(SparkPort.LEFT_DRIVE1),
                                                                  new Spark(SparkPort.LEFT_DRIVE2));

            SpeedController rightDrive = new MultiSpeedController(new Spark(SparkPort.RIGHT_DRIVE3),
                                                                  new Spark(SparkPort.RIGHT_DRIVE4));

            m_rawDifferentialDrive = new DifferentialDrive(leftDrive, rightDrive);
            m_teleopTransDrive = new TeleopTransDrive(m_rawDifferentialDrive, m_transmission);
        } catch(Exception ex) {
            DriverStation.reportError("Could not instantiate Drive Train Motors\n", false);
        }

        /* 
        try {
            encoder = new Encoder(EncoderPort.One_A, EncoderPort.One_B, true);
        } catch(Exception e) {
            DriverStation.reportError("Could not instantiate Encoder: One_A\n", false);
        }
        */
    }
    
    public void periodic(Joystick stick, Double abs_limit) {
        if(stick == null) {
            DriverStation.reportError("No Joystick, cannot run Chassis periodic\n", false);
            return;
        }

        if (m_teleopTransDrive != null) {
            m_teleopTransDrive.arcadeDrive(stick, abs_limit); // m_drive with arcade style
        }

        if (m_compressor != null) {
            m_compressor.setClosedLoopControl(true);
        }

        m_rawDifferentialDrive.arcadeDrive(stick.getY(), -stick.getX());
    }

    public void stop() {
        m_rawDifferentialDrive.arcadeDrive(0.0, 0.0);        
    }

    private DifferentialDrive m_rawDifferentialDrive;
    private Solenoidal m_transmission;
    private TeleopTransDrive m_teleopTransDrive;
    private Compressor m_compressor;
    private SerialPort m_arduino;
    
    //private TeleopTransDrive m_drive;
    //private Encoder encoder;
}
