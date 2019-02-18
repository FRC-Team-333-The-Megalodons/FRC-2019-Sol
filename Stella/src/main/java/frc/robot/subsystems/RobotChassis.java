package frc.robot.subsystems;

//import frc.robot.Map.EncoderPort;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.subsystems.*;
import frc.robot.subsystems.RobotMap.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;

public class RobotChassis {

    private DifferentialDrive m_rawDifferentialDrive;
    private Solenoidal m_transmission;
    private Compressor m_compressor;
    private TeleopTransDrive m_teleopTransDrive;
    private CANSparkMax m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower;
    private CANEncoder m_leftLeaderEnc, m_leftFollowerEnc, m_rightLeaderEnc, m_rightFollowerEnc;
    private LimelightDrive m_limelightDrive;
    private AnalogInput m_ultrasonic;
    // private SerialPort m_arduino;
    private NetworkTable m_networkTable;
    private double tx, ty, area;

    public RobotChassis(RobotHatchGrab m_hatchGrab) {

        // Instantiate the compressor
        try {
            m_compressor = new Compressor(RobotMap.CompressorPort.MAIN_COMPRESSOR);
        } catch (Exception ex) {
            DriverStation.reportError("Could not start Compressor\n", false);
        }

        // Instantiate Drive Train Motors, Transmission, and also the Wrapper Drives
        try {
            m_transmission = new Solenoidal(SolenoidPort.DRIVE_TRANS_1, SolenoidPort.DRIVE_TRANS_2);
            
            double rampRate =SmartDashboard.getNumber("ramp rate", .25);

            m_leftLeader = new CANSparkMax(CANSparkID.LEFT_LEADER, MotorType.kBrushless);    
            m_leftLeaderEnc = m_leftLeader.getEncoder();
            m_leftFollower = new CANSparkMax(CANSparkID.LEFT_FOLLOWER, MotorType.kBrushless);
            m_leftFollowerEnc = m_leftFollower.getEncoder();
            m_leftFollower.follow(m_leftLeader);
            m_leftLeader.setRampRate(rampRate);
            //m_leftLeader.setInverted(true);


            m_rightLeader = new CANSparkMax(CANSparkID.RIGHT_LEADER, MotorType.kBrushless);
            m_rightLeaderEnc = m_rightLeader.getEncoder();
            m_rightFollower = new CANSparkMax(CANSparkID.RIGHT_FOLLOWER, MotorType.kBrushless);
            m_rightFollowerEnc = m_rightFollower.getEncoder();
            m_rightFollower.follow(m_rightLeader);
            m_rightLeader.setRampRate(rampRate);

            m_rawDifferentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
            //m_rawDifferentialDrive = new DifferentialDrive(CANLeftLeader, CANRightLeader);      //MEANT FOR CAN!
            m_teleopTransDrive = new TeleopTransDrive(m_rawDifferentialDrive, m_transmission, PlayerButton.FORCE_LOW_TRANSMISSION);
            m_limelightDrive = new LimelightDrive(m_rawDifferentialDrive, m_transmission, m_hatchGrab);
        } catch (Exception ex) {
              DriverStation.reportError("Could not instantiate Drive Train Motors\n", false);
        }

        try {
            m_ultrasonic = new AnalogInput(AnalogPort.ULTRASONIC_SENSOR);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate Ultrasonic sensor\n", false);

        }

        // Instantiate the Limelight's Network Tables
        try {
            m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        } catch (Exception ex) {
            DriverStation.reportError("Could not set up limelight network tables\n", false);
        }

        /*
         * try { encoder = new Encoder(EncoderPort.One_A, EncoderPort.One_B, true); }
         * catch(Exception e) {
         * DriverStation.reportError("Could not instantiate Encoder: One_A\n", false); }
         */
    }

    public void periodic(Joystick stick, Double abs_limit) {
        if (stick == null) {
            DriverStation.reportError("No Joystick, cannot run Chassis periodic\n", false);
            return;
        }

        if (stick.getRawButton(PlayerButton.CHASE_REFLECTIVE_TAPE)) {
            double cap = SmartDashboard.getNumber("AutoDriveSpeedCap", 0.5f);
            m_limelightDrive.autoDrive(tx, ty, area, cap);
          //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);  //Turn on LED on limelight
        } else {
          //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  //Turn off LED on limelight
            m_teleopTransDrive.curvatureDrive(stick, abs_limit, stick.getRawButton(PlayerButton.FORCE_NO_CURVATURE)); // m_drive with arcade style
           //m_teleopTransDrive.arcadeDrive(stick, abs_limit);
        }

        


        if (m_compressor != null) {
            m_compressor.setClosedLoopControl(true);
        }
    }

    public void updateLatestVisionTargets() {
        tx = m_networkTable.getEntry("tx").getDouble(0.0);
        ty = m_networkTable.getEntry("ty").getDouble(0.0);
        area = m_networkTable.getEntry("ta").getDouble(0.0);
    }


    public void updateDashboard()
    {
        SmartDashboard.putNumber("LeftLeader:", m_leftLeaderEnc.getPosition());
        SmartDashboard.putNumber("LeftFollower:", m_leftFollowerEnc.getPosition());
        SmartDashboard.putNumber("RightLeader:", m_rightLeaderEnc.getPosition());
        SmartDashboard.putNumber("RightFollower:", m_rightFollowerEnc.getPosition());
        SmartDashboard.putNumber("ultrasonic_avg:", m_ultrasonic.getAverageVoltage()*39);
        SmartDashboard.putNumber("ultrasonic:", m_ultrasonic.getVoltage()*39);
        SmartDashboard.putNumber("raw analog 0:", m_ultrasonic.getValue());
        SmartDashboard.putNumber("Limelight X", tx);
        SmartDashboard.putNumber("Limelight Y", ty);
        SmartDashboard.putNumber("Limelight Area", area);
    }

    public void stop() {
        m_rawDifferentialDrive.arcadeDrive(0.0, 0.0);
    }

}