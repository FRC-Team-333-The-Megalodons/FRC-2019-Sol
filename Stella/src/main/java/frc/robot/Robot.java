/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.RobotMap.JoystickPort;
import edu.wpi.first.wpilibj.DriverStation;

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

        /* Chassis */
        try {
            m_chassis = new RobotChassis();
        } catch (Exception ex) {
            DriverStation.reportError("Couldnt instantiate Chassis", false);
        }

        /* Mech */
        try {
            m_mech = new RobotMech();
        } catch (Exception ex) {
            DriverStation.reportError("Couldn't instantiate Mech", false);
        }

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
        // Autonomous just calls teleop.
        teleopInit();
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
    public void autonomousPeriodic() {
        // Autonomous just calls teleop.
        teleopPeriodic();
        /*
        switch (m_autoSelected) {
        case kCustomAuto:
            // Put custom auto code here
            break;
        case kDefaultAuto:
        default:
            // Put default auto code here
            break;
        }
        */
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        if (!isEnabled()) { return; }

        Timer.delay(0.005);
        m_chassis.periodic(m_driverJoystick, 1.0);
        m_mech.periodic(m_driverJoystick);
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
        SmartDashboard.setDefaultNumber("AutoDriveSpeedCap", 0.6f);
        SmartDashboard.setDefaultNumber("AutoDrive_kAIM", 0.7f);
        SmartDashboard.setDefaultNumber("AutoDrive_kDistance", 1.0f);
        SmartDashboard.setDefaultNumber("AutoDrive_minInc", 0.05f);
    }
}
