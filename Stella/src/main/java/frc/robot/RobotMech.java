package frc.robot;

import frc.robot.RobotMap.DigitalInputPort;
import frc.robot.RobotMap.PlayerButton;
import frc.robot.RobotMap.SolenoidPort;
import frc.robot.RobotMap.TalonPort;
import edu.wpi.first.wpilibj.DoubleSolenoid;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotMech {

    public static final double MOTOR_POWER = 0.4;
    public static final double SHOOTER_POWER = -1.0;
    public static final double INTAKE_POWER = .5;

    public static final boolean CARGO_IN = false;
    public static final boolean CARGO_OUT = true;

    private RobotArm m_intakeArm;
    private RobotRoller m_roller;
    private RobotShooter m_shooter;
    // private TalonSRX talon;
    private SolenoidT m_hatchGrab;
    private DoubleSolenoid m_rollerSolenoids;
    private RobotCargoState m_cargoState;
    private RobotArmState m_upperArmLimit, m_lowerArmLimit;

    public RobotMech() {
        /* Roller */
        try {
            m_roller = new RobotRoller(TalonPort.Roller);
        } catch (Exception e) {

            DriverStation.reportError("Couldn't instantiate Roller\n", false);
        }

        /* Arm */
        try {
            m_intakeArm = new RobotArm(TalonPort.IntakeArm);
        } catch (Exception e) {
            DriverStation.reportError("Couldn't instantiate Arm\n", false);
        }

        /* Shooter */
        try {
            m_shooter = new RobotShooter(TalonPort.Shooter);
        } catch (Exception e) {
            DriverStation.reportError("Couldn't instantiate Shooter\n", false);
        }

        try {
            m_cargoState = new RobotCargoState(DigitalInputPort.CLAW_SWITCH);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate claw limit switch\n", false);
        }

        try {
            m_upperArmLimit = new RobotArmState(DigitalInputPort.UPPER_ARM_SWITCH);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate upper arm limit switch\n", false);
        }

        try {
            m_lowerArmLimit = new RobotArmState(DigitalInputPort.LOWER_ARM_SWITCH);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate lower arm limit switch\n", false);
        }

        /* Instantiate the Roller solenoids */
        try {
            m_rollerSolenoids = new DoubleSolenoid(SolenoidPort.ROLLER_UP, SolenoidPort.ROLLER_DOWN);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the roller solenoids\n", false);
        }

        /* Instantiate the Hatch grab */
        try {
            m_hatchGrab = new SolenoidT(SolenoidPort.HATCH_GRAB);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate hatch grab mechanism\n", false);
        }
    }

    public void periodic(Joystick stick) {
        if (stick == null) {
            DriverStation.reportError("No Joystick, cannot run Mech periodic\n", false);
            return;
        }

        // ARM : PWM 6
        // 7: Raise arm up
        // 9: Lower arm down
        if (stick.getRawButton(PlayerButton.ARM_UP) && !m_upperArmLimit.isArmAtLimit()) {
            m_intakeArm.moveArmUp();
        } else if (stick.getRawButton(PlayerButton.ARM_DOWN) && !m_lowerArmLimit.isArmAtLimit()) {
            m_intakeArm.moveArmDown();
        } else {
            m_intakeArm.stopArm();
        }

        // ROLLER : PWM 6
        // 3: Take cargo in
        // 5: Give ball out
        if (stick.getRawButton(PlayerButton.INTAKE)) {
            if (m_cargoState.isCargoPresent()) {
                m_roller.stopRoller();
            } else {
                m_roller.pullRollerIn();
            }
        } else if (stick.getRawButton(PlayerButton.PUSH_ROLLER_OUT)) {
            m_roller.pushRollerOut();
        } else {
            m_roller.stopRoller();
        }

        SmartDashboard.putBoolean("Is Cargo Present?", m_cargoState.isCargoPresent());
        SmartDashboard.putBoolean("Is Arm At Upper Limit?", m_upperArmLimit.isArmAtLimit());
        SmartDashboard.putBoolean("Is Arm At Lower Limit?", m_lowerArmLimit.isArmAtLimit());

        // ROLLER_SOLENOIDS : PCM 3, 4
        // 11: solenoid toggle
        if (stick.getRawButton(PlayerButton.ROLLER_TOGGLE) && enoughTimePassed(System.currentTimeMillis(), initTime)) {
            if (m_rollerSolenoids.get() == Value.kReverse) {
                m_rollerSolenoids.set(Value.kForward);
            } else {
                m_rollerSolenoids.set(Value.kReverse);
            }
            initTime = System.currentTimeMillis();
        }

        // SHOOTER : PWM 7
        // 3: Take cargo in
        // 4: Shoot ball out
        if (stick.getRawButton(PlayerButton.SHOOTER)) {
            m_shooter.fireShooter();
        } else if (stick.getRawButton(PlayerButton.INTAKE) && !m_cargoState.isCargoPresent()) {
            m_shooter.intakeShooter();
        } else {
            m_shooter.stopShooter();
        }

        // HATCH_GRAB : PCM 5
        // Trigger: Unlock
        if (stick.getTrigger()) {
            m_hatchGrab.set(true);
        } else {
            m_hatchGrab.set(false);
        }

    }

    private boolean enoughTimePassed(long timeNow, long initTime) {
        return timeNow > initTime + 1000;
    }

    long initTime = System.currentTimeMillis();
}
