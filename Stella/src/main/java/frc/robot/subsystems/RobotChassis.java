package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Robot;
import frc.robot.subsystems.RobotMap.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotChassis {

    private DifferentialDrive m_rawDifferentialDrive;
    private Solenoidal m_transmission;
    private Compressor m_compressor;
    private TeleopTransDrive m_teleopTransDrive;
   // private CANSparkMax m_rightClimber, m_climber;
    private CANSparkMax m_leftLeader, m_leftFollower, m_leftFollower2, m_rightLeader, m_rightFollower, m_rightFollower2;
    private CANEncoder m_leftLeaderEnc, m_leftFollowerEnc, m_leftFollowerEnc2, m_rightLeaderEnc, m_rightFollowerEnc, m_rightFollowerEnc2;
    private CANEncoder m_climberEnc;
    private LimelightDrive m_limelightDrive;
    // private SerialPort m_arduino;
    private NetworkTable m_networkTable;
    private double m_tx, m_ty, m_area, m_skew, m_corner;
    private NetworkTableEntry m_pipeline;
    private IdleMode m_lastIdleMode;
    private AutonArcDrive m_autonArcDrive;
    private RobotUtils.LimelightLED m_led;
    //private AutonTurnDrive m_autonTurnDrive;

    public RobotChassis(NetworkTable networkTable, NetworkTableEntry pipeline, RobotHatchGrab hatchGrab, RobotArm arm, RobotUtils.LimelightLED led) {
        m_led = led;
        // Instantiate the compressor
        try {
            m_compressor = new Compressor(RobotMap.CompressorPort.MAIN_COMPRESSOR);
        } catch (Exception ex) {
            DriverStation.reportError("Could not start Compressor\n", false);
        }

        
        m_networkTable = networkTable;
        m_pipeline = pipeline;

        // Instantiate Drive Train Motors, Transmission, and also the Wrapper Drives
        try {
            m_transmission = new Solenoidal(SolenoidPort.DRIVE_TRANS_1, SolenoidPort.DRIVE_TRANS_2);
            
            double rampRate =SmartDashboard.getNumber("ramp rate", .25);

            m_leftLeader = new CANSparkMax(CANSparkID.LEFT_LEADER, MotorType.kBrushless);    
            m_leftLeaderEnc = m_leftLeader.getEncoder();

            m_leftFollower = new CANSparkMax(CANSparkID.LEFT_FOLLOWER, MotorType.kBrushless);
            m_leftFollowerEnc = m_leftFollower.getEncoder();
            m_leftFollower.follow(m_leftLeader);

            m_leftFollower2 = new CANSparkMax(CANSparkID.LEFT_FOLLOWER2, MotorType.kBrushless);
            m_leftFollowerEnc2 = m_leftFollower2.getEncoder();
            m_leftFollower2.follow(m_leftLeader);

            m_leftLeader.setRampRate(rampRate);
            //m_leftLeader.setInverted(true);


            m_rightLeader = new CANSparkMax(CANSparkID.RIGHT_LEADER, MotorType.kBrushless);
            m_rightLeaderEnc = m_rightLeader.getEncoder();

            m_rightFollower = new CANSparkMax(CANSparkID.RIGHT_FOLLOWER, MotorType.kBrushless);
            m_rightFollowerEnc = m_rightFollower.getEncoder();
            m_rightFollower.follow(m_rightLeader);

            m_rightFollower2 = new CANSparkMax(CANSparkID.RIGHT_FOLLOWER2, MotorType.kBrushless);
            m_rightFollowerEnc2 = m_rightFollower2.getEncoder();
            m_rightFollower2.follow(m_rightLeader);

            m_rightLeader.setRampRate(rampRate);

            m_rawDifferentialDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);
            //m_rawDifferentialDrive = new DifferentialDrive(CANLeftLeader, CANRightLeader);      //MEANT FOR CAN!
            m_teleopTransDrive = new TeleopTransDrive(m_rawDifferentialDrive, m_transmission, PlayerButton.FORCE_LOW_TRANSMISSION, true);
            m_limelightDrive = new LimelightDrive(m_rawDifferentialDrive, m_transmission, hatchGrab, arm.getCargoState());
            
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Drive Train Motors\n", false);
        }

     /*   try {
            m_rightClimber = new CANSparkMax(CANSparkID.RIGHT_CLIMBER, MotorType.kBrushless);
            m_climber = new CANSparkMax(CANSparkID.LEFT_CLIMBER, MotorType.kBrushless);
            m_rightClimber.follow(m_climber, true);
            m_climberEnc = m_rightClimber.getEncoder();
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Climber\n", false);
        }
*/
       /* try {
            m_ultrasonic = new AnalogInput(AnalogPort.ULTRASONIC_SENSOR);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate Ultrasonic sensor\n", false);
         }
*/


        /*
         * try { encoder = new Encoder(EncoderPort.One_A, EncoderPort.One_B, true); }
         * catch(Exception e) {
         * DriverStation.reportError("Could not instantiate Encoder: One_A\n", false); }
         */
    }

    public boolean lowTransmission()
    {
        return m_teleopTransDrive.lowTransmission();
    }

    public boolean highTransmission()
    {
        return m_teleopTransDrive.highTransmission();
    }

    public CANSparkMax getLeftLeaderNeo()
    {
        return m_leftLeader;
    }

    public CANSparkMax getRightLeaderNeo()
    {
        return m_rightLeader; 
    }

    public IdleMode periodic(Joystick stick, Double abs_limit, boolean sandstorm) {
        IdleMode idleMode = IdleMode.kCoast;

        if (stick == null) {
            DriverStation.reportError("No Joystick, cannot run Chassis periodic\n", false);
            return idleMode;
        }
        boolean chase_hatch = Robot.is_limelight_chase(stick, m_led);
        boolean auton_drive = sandstorm && (stick.getRawButton(PlayerButton.INTAKE_CARGO_HUMAN_1) ||
                                            stick.getRawButton(PlayerButton.INTAKE_CARGO_HUMAN_2));
        if (chase_hatch) {
            double cap = 0.6; //SmartDashboard.getNumber("AutoDriveSpeedCap", 0.5f);
            int pipeline_index = /* chase_cargo ? RobotMap.LimelightPipeline.CARGO :*/ RobotMap.LimelightPipeline.HATCH;
            RobotUtils.updateLimelightPipeline(m_pipeline, pipeline_index);

            boolean rocketMode = stick.getRawButton(PlayerButton.ROCKET_MODE);
            // || stick.getRawButton(PlayerButton.ROCKET_MODE_2));
            m_limelightDrive.autoDrive(pipeline_index, m_tx, m_ty, m_area, rocketMode);
        } else if (auton_drive) {
            idleMode = IdleMode.kBrake;

            if (m_autonArcDrive == null) {
                double leftTarget = -56.28, rightTarget = 53.64;
                double leftCap    =   0.7, rightCap    = 0.6;

                if (stick.getRawButton(PlayerButton.INTAKE_CARGO_HUMAN_2)) {
                    leftTarget = -72.57;
                    rightTarget = 66.52;
                    leftCap    =   0.75;
                    rightCap    =  0.55;
                }
                // for Cargo Hatch:
                //   - Left cap at 0.7, Right cap at 0.6
                //   - Left = -56.3, right = 53.6

                // for Rocket hatch
                //   - Left cap at 0.8, Right cap at 0.6
                //   - Left = -72.57, right = 66.52
                m_autonArcDrive = new AutonArcDrive(m_leftLeader, m_rightLeader, leftTarget, rightTarget, leftCap, rightCap);

            }

            m_autonArcDrive.periodic();

            // After the turn, for the Cargo Ship Hatch, we need to turn to left=-50.5, right=60.57
        } else {
            //m_teleopTransDrive.curvatureDrive(stick, abs_limit); // m_drive with arcade style
            m_teleopTransDrive.arcadeDrive(stick, abs_limit);
        }   

        if (m_compressor != null) {
            m_compressor.setClosedLoopControl(true);
        }

        return idleMode;
    }

    public void updateLatestVisionTargets() {
        m_tx = m_networkTable.getEntry("tx").getDouble(0.0);
        m_ty = m_networkTable.getEntry("ty").getDouble(0.0);
        m_area = m_networkTable.getEntry("ta").getDouble(0.0);
        m_skew = m_networkTable.getEntry("ts").getDouble(0.0);
    }


    public void updateDashboard()
    {
        SmartDashboard.putNumber("LeftLeader:", m_leftLeaderEnc.getPosition());
        SmartDashboard.putNumber("LeftFollower:", m_leftFollowerEnc.getPosition());
        SmartDashboard.putNumber("RightLeader:", m_rightLeaderEnc.getPosition());
        SmartDashboard.putNumber("RightFollower:", m_rightFollowerEnc.getPosition());
        //SmartDashboard.putNumber("ultrasonic_avg:", m_ultrasonic.getAverageVoltage()*39);
        //SmartDashboard.putNumber("ultrasonic:", m_ultrasonic.getVoltage()*39);
       // SmartDashboard.putNumber("raw analog 0:", m_ultrasonic.getValue());
        SmartDashboard.putNumber("Limelight X", m_tx);
        SmartDashboard.putNumber("Limelight Y", m_ty);
        SmartDashboard.putNumber("Limelight Area", m_area);
        SmartDashboard.putNumber("Limelight Skew", m_skew);
    }

    public void stop() {
        m_rawDifferentialDrive.arcadeDrive(0.0, 0.0);
    }

    public void setIdleMode(IdleMode mode)
    {
        if (m_lastIdleMode == null || m_lastIdleMode != mode) {
            System.out.println("Setting idle mode to "+mode);
            m_lastIdleMode = mode;
            m_leftLeader.setIdleMode(mode);
            m_leftFollower.setIdleMode(mode);
            m_leftFollower2.setIdleMode(mode);
            m_rightLeader.setIdleMode(mode);
            m_rightFollower.setIdleMode(mode);
            m_rightFollower2.setIdleMode(mode);
        }
    }

}
