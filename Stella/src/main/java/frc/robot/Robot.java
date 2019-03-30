/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;
import frc.robot.subsystems.RobotMap.*;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /*
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    */
    private RobotChassis m_chassis;
    private RobotMech m_mech;
    private Joystick m_driverJoystick;
    private NetworkTable m_networkTable = null;
    private NetworkTableEntry m_limelightLedMode = null;
    private boolean m_lastHatchLEDstate = false;
    int m_lastLimelightLedMode = -1;

    //AnalogInput ultrasonic;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        /*
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);
        */

        /* Joystick */
        try {
            m_driverJoystick = new Joystick(JoystickPort.Joystick_Port);
        } catch (Exception ex) {
            DriverStation.reportError("Couldn't instantiate Joystick", false);
        }

        /* Limelight */
        NetworkTableEntry pipeline = null;
        try {
            m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
            pipeline = m_networkTable.getEntry("pipeline");
            pipeline.setNumber(RobotMap.LimelightPipeline.HATCH); // Default to 0 (hatch_333)
            m_limelightLedMode = m_networkTable.getEntry("ledMode");
        } catch (Exception ex) {
            DriverStation.reportError("Could not set up limelight network tables\n", false);
        }

        /* Mech */
        try {
            m_mech = new RobotMech(pipeline);
        } catch (Exception ex) {
            for (int i = 0; i < ex.getStackTrace().length; i++) {
                System.out.println(i+" : "+ex.getStackTrace()[i]);
            }
            DriverStation.reportError("Couldn't instantiate Mech", false);
        }

        /* Chassis */
        try {
            m_chassis = new RobotChassis(m_networkTable, pipeline, m_mech.getHatchGrab(), m_mech.getRobotArm());
            m_chassis.setIdleMode(IdleMode.kCoast);
            m_mech.setChassis(m_chassis);
        } catch (Exception ex) {
            for (int i = 0; i < ex.getStackTrace().length; i++) {
                System.out.println(i+" : "+ex.getStackTrace()[i]);
            }
            DriverStation.reportError("Couldnt instantiate Chassis", false);
        }

        try {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(160, 120); //320, 240);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate driver camera\n", false);

        }

/*
        try {
            ultrasonic = new AnalogInput(0);
        } catch (Exception ex) {
            DriverStation.reportError("Zach's Sensor no worky...", false);
        }
*/
        initDashboard();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want ran during disabled, autonomous,
     * teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        m_chassis.updateLatestVisionTargets();
        m_mech.updateDashboard();
        m_chassis.updateDashboard();
        SmartDashboard.putNumber("Joystick X", m_driverJoystick.getX());
        SmartDashboard.putNumber("Joystick Y", m_driverJoystick.getY());
        SmartDashboard.putNumber("Joystick Z", m_driverJoystick.getZ());
        SmartDashboard.putNumber("Joystick Throttle", m_driverJoystick.getThrottle());
        SmartDashboard.putNumber("Joystick POV", m_driverJoystick.getPOV());
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable chooser
     * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
     * remove all of the chooser code and uncomment the getString line to get the
     * auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure below with additional strings. If using the SendableChooser
     * make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        // Autonomous just pushes out the nose.

        m_mech.pushNoseOut();

        /*
        m_autoSelected = m_chooser.getSelected();
        // autoSelected = SmartDashboard.getString("Auto Selector",
        // defaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
        */
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() 
    {
        // Autonomous just calls teleop.
        teleopPeriodic(true);
    }

    @Override
    public void teleopInit()
    {
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        teleopPeriodic(false);
    }

    public void teleopPeriodic(boolean sandstorm)
    {
        if (!isEnabled()) { return; }

        Timer.delay(0.005);
        
        IdleMode idleMode = IdleMode.kCoast;
        
        // This order is important: Mech first, as it will tell us whether it's
        //  temporarily taking control of the DriveTrain motors (needed for some control sequences)
        boolean ignoreJoystickInChassis = m_mech.periodic(m_driverJoystick, sandstorm);
        
        // This admittedly gets weird: If the Mech has told us it's taking control
        //  of the DriveTrain motors, we should not do Periodic on the Chassis,
        //  and we should force into Brake Mode and Low Transmission to let it do
        //  its thing.
        if (ignoreJoystickInChassis) {
            m_chassis.lowTransmission();
            idleMode = IdleMode.kBrake;
        } else {
            idleMode = m_chassis.periodic(m_driverJoystick, 1.0, sandstorm);
        }
        
        /*** SPECIAL STUFF FOR DEFENSE MODE ***/
        if (m_driverJoystick.getThrottle() < 0) {
            // set idle to brake
            // TODO: Why not be in Brake if in Defense during Sandstorm?
            if (!sandstorm) {
                idleMode = IdleMode.kBrake;
            }
            if (m_lastLimelightLedMode != LimelightLEDMode.OFF) {
                m_limelightLedMode.setNumber(LimelightLEDMode.OFF);
                m_lastLimelightLedMode = LimelightLEDMode.OFF;
            }
            if (!m_lastHatchLEDstate) {
                m_mech.getHatchGrab().activateIndicatorLight();
                m_lastHatchLEDstate = true;
            }
        } else {
            if (m_lastLimelightLedMode != LimelightLEDMode.PIPELINE) {
                m_limelightLedMode.setNumber(LimelightLEDMode.PIPELINE);
                m_lastLimelightLedMode = LimelightLEDMode.PIPELINE;
            }
            if (m_lastHatchLEDstate) {
                m_mech.getHatchGrab().deactivateIndicatorLight();
                m_lastHatchLEDstate = false;
            }
        }

        m_chassis.setIdleMode(idleMode);

    //  System.out.println("getValue: "+ultrasonic.getAverageValue());
    //  System.out.println("getVoltage: "+ultrasonic.getAverageVoltage());
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        
        // Nothing here yet.
    }


    public void initDashboard()
    {
        //SmartDashboard.setDefaultNumber("AutoDriveSpeedCap", 0.6f);
        //SmartDashboard.setDefaultNumber("AutoDrive_kAIM", 0.7f);
        //SmartDashboard.setDefaultNumber("AutoDrive_kDistance", 1.0f);
        //SmartDashboard.setDefaultNumber("AutoDrive_minInc", 0.05f);
    }
}