package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

//import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.RobotMap.*;
import frc.robot.controllers.*;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;

public class RobotMech {

    public static final boolean CARGO_IN = false;
    public static final boolean CARGO_OUT = true;

    private RobotArm m_arm;
    private RobotRoller m_roller;
    private RobotShooter m_shooter;
    private RobotHatchGrab m_hatchGrab;
    private DoubleSolenoid m_rollerSolenoids;
    private boolean m_noseState;
    private long m_lastNoseStateChange;
    private Long m_lastShootOutChange;
    private Long m_lastShootersInChange;
    private IntakeCargoFromFloor m_intakeCargoFromFloor;
    private IntakeCargoFromHuman m_intakeCargoFromHuman;
    private ShootCargoInShipFace m_shootCargoInShipFace;
    private EjectCargoToFloor m_ejectCargoToFloor;
    private ShootCargoIntoShip m_shootCargoIntoShip;
    private ActivateDefenseMode m_activateDefenseMode;
    private DigitalInput m_intakeOutLimitSwitch;

  //  public double armPos = m_armPotentiometer.get();


    public RobotMech(NetworkTableEntry pipeline) {
        /* Roller */
        try {
            m_roller = new RobotRoller(CANSparkID.INTAKE_NEO);
        } catch (Exception e) {

            DriverStation.reportError("Couldn't instantiate Roller\n", false);
        }

        /* Arm */
        try {
            m_arm = new RobotArm(CANSparkID.ARM_NEO);
        } catch (Exception e) {
            DriverStation.reportError("Couldn't instantiate Arm\n", false);
        }

        /* Shooter */
        try {
            m_shooter = new RobotShooter(CANSparkID.SHOOTER_TOP, MotorType.kBrushed, CANSparkID.SHOOTER_BOTTOM, MotorType.kBrushed);
        } catch (Exception e) {
            DriverStation.reportError("Couldn't instantiate Shooter\n", false);
        }

        /* Instantiate the Roller solenoids */
        try {
            m_rollerSolenoids = new DoubleSolenoid(SolenoidPort.INTAKE_ROLLER_IN, SolenoidPort.INTAKE_ROLLER_OUT);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the roller solenoids\n", false);
        }

        /* Instantiate the Hatch grab */
        try {
            m_hatchGrab = new RobotHatchGrab(SolenoidPort.HATCH_GRAB, AnalogPort.LEFT_PANEL_SENSOR, AnalogPort.RIGHT_PANEL_SENSOR, SolenoidPort.PANEL_INDICATOR_LIGHT);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate hatch grab mechanism\n", false);
        }

        try {
            m_intakeOutLimitSwitch = new DigitalInput(DigitalInputPort.INTAKE_OUT_SWITCH);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instant limit switch for intake in outward position\n", false);
        }

        /* Controllers */
        m_intakeCargoFromFloor = new IntakeCargoFromFloor(this, m_arm);
        m_intakeCargoFromHuman = new IntakeCargoFromHuman(this, m_arm);
        m_shootCargoInShipFace = new ShootCargoInShipFace(this, m_arm);
        m_shootCargoIntoShip   = new ShootCargoIntoShip(this, m_arm);
        m_activateDefenseMode  = new ActivateDefenseMode(this, m_arm);
        m_ejectCargoToFloor    = new EjectCargoToFloor(this, m_arm);
    }

    public RobotArm getRobotArm()
    {
        return m_arm;
    }

    public boolean hasPanel()
    {
        return m_hatchGrab.IsPanelOnLeft();
    }

    public void updateDashboard() {   
        m_arm.updateDashboard();
        SmartDashboard.putBoolean("Is nose out:", isNoseActuallyOut());
        SmartDashboard.putBoolean("hatchSensor", hasPanel());
        SmartDashboard.putNumber("hatch sensor raw voltage", m_hatchGrab.RawValue());
    }

    public void pushNoseOut()
    {
        m_rollerSolenoids.set(Value.kReverse);

        if (!isNoseOut()) {
            m_lastNoseStateChange = System.currentTimeMillis();
            m_noseState = true;
        }
    }

    public void pullNoseIn()
    {
        m_rollerSolenoids.set(Value.kForward);

        if (isNoseOut()) {
            m_lastNoseStateChange = System.currentTimeMillis();
            m_noseState = false;
        }
    }

    public boolean isNoseOut()
    {
        return m_noseState;
    }

    public long timeSinceLastNoseStateChange()
    {
        return System.currentTimeMillis() - m_lastNoseStateChange;
    }

    public boolean wasCargoRecentlyShot()
    {
        // No choice but to do this based on time.
        final double SHOOT_TIME = 1000;

        if (m_lastShootOutChange != null) {
            return (System.currentTimeMillis() - m_lastShootOutChange <= SHOOT_TIME);
        }
        return false;
    }

    public boolean wasCargoRecentlyConsumed()
    {
        final double CONSUME_TIME = 500;

        if (m_lastShootersInChange != null) {
            return (System.currentTimeMillis() - m_lastShootersInChange <= CONSUME_TIME);
        }
        return false;
    }

    public boolean isNoseActuallyOut()
    {
        return !m_intakeOutLimitSwitch.get();
        /*
        // At some point, this will be replaced by a Limit Switch that will actually tell us the position of the Nose.
        // For now, though, we'll just see whether the last command was to push out, and whether sufficient time (1 second?) has passed.
        long minimum_time_since_nose_out = 1000;
        return (isNoseOut() && timeSinceLastNoseStateChange() >= minimum_time_since_nose_out);
        */
    }

    public void periodic(Joystick stick) {
        if (stick == null) {
            DriverStation.reportError("No Joystick, cannot run Mech periodic\n", false);
            return;
        }


        if (hasPanel()) {
            m_hatchGrab.activateIndicatorLight();
        }

        if (stick.getThrottle() < 0) { // Defense mode! Activate defense mode.
            //RobotUtils.updateLimelightPipeline(m_pipeline, RobotMap.LimelightPipeline.CARGO);
            if (m_activateDefenseMode.do_defense()) {
                if (stick.getTrigger() && !m_arm.getCargoState().isCargoPresent()) {
                    pullInShooterRollers();
                } else {
                    if (stick.getRawButton(PlayerButton.EJECT_CARGO)) {
                      pushOutShooterRollers(RobotShooter.FULL_SHOOTER_POWER);
                    } else {
                      stopShooterRollers();
                    }
                }
            }
            return;
        } 

        //int limelight_pipeline = RobotMap.LimelightPipeline.HATCH;

        boolean is_hatch_already_governed = false;
        if (m_arm.getCargoState().isCargoPresent()) {
            m_hatchGrab.close();
            is_hatch_already_governed = true;
        }

        boolean is_controller_invoked = false;
        if (stick.getRawButton(PlayerButton.INTAKE_CARGO_FLOOR_1) ||
            stick.getRawButton(PlayerButton.INTAKE_CARGO_FLOOR_2)) {
            //limelight_pipeline = RobotMap.LimelightPipeline.CARGO;
            m_intakeCargoFromFloor.do_intake();
            is_controller_invoked = true;
        } else if (stick.getRawButton(PlayerButton.INTAKE_CARGO_HUMAN_1) ||
                   stick.getRawButton(PlayerButton.INTAKE_CARGO_HUMAN_2)) {
            m_intakeCargoFromHuman.do_intake();
            is_controller_invoked = true;
        } else if (stick.getRawButton(PlayerButton.EJECT_CARGO)) {
            m_ejectCargoToFloor.do_eject();
            is_controller_invoked = true;
        } else {
            if (m_arm.getCargoState().isCargoPresent() || wasCargoRecentlyShot()) {
                if (stick.getTrigger()) {
                    // If they're holding the Rocket Shot buttons, instead user the lower height.
                    boolean rocketShot = ((stick.getRawButton(PlayerButton.ROCKET_MODE_1) ||
                     (stick.getRawButton(PlayerButton.ROCKET_MODE_2))));
                    double position = (rocketShot ? RobotArm.ROCKET_SHOOTING_POS : RobotArm.SHOOTING_POSITION);
                    double power    = (rocketShot ? RobotShooter.ROCKET_SHOOTER_POWER : RobotShooter.FULL_SHOOTER_POWER);
                    m_shootCargoIntoShip.do_shoot(position, power);
                    is_controller_invoked = true;
                }
            } else if (!is_hatch_already_governed) {
                if (stick.getTrigger()) {
                    m_hatchGrab.close();
                } else {
                    m_hatchGrab.open();
                }
            }
        }

        //RobotUtils.updateLimelightPipeline(m_pipeline, limelight_pipeline);

        boolean isPressingRollerEjectButton = stick.getRawButton(PlayerButton.EJECT_ROLLER);

        if (!is_controller_invoked) {
            m_arm.stopArm();
            if (isPressingRollerEjectButton) {
                pushOutIntakeRollers();
            } else {
                stopIntakeRollers();
            }
            stopShooterRollers();
        }
    }

    public RobotHatchGrab getHatchGrab()
    {
        return m_hatchGrab;
    }

    public void pushOutIntakeRollers()
    {
        m_roller.pushRollerOut();
    }

    public void stopShooterRollers()
    {
        m_lastShootOutChange = null;
        m_lastShootersInChange = null;
        m_shooter.stopShooter();
    }

    public void stopIntakeRollers()
    {
        m_roller.stopRoller();
    }

    public void pullInShooterRollers()
    {
        m_lastShootOutChange = null;
        if (m_lastShootersInChange == null) {
          m_lastShootersInChange = System.currentTimeMillis();
        }
        m_shooter.intakeShooter();
    }

    public void pushOutShooterRollers(double power)
    {
        if (m_lastShootOutChange == null) {
            m_lastShootOutChange = System.currentTimeMillis();
        }
        m_lastShootersInChange = null;
        m_shooter.fireShooter(power);
    }

    public void pullInIntakeRollers()
    {
        m_roller.pullRollerIn();
    }

    public boolean isCargoPresent()
    {
        return m_arm.isCargoPresent();
    }

    long initTime = System.currentTimeMillis();
}