package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

//import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.*;
import frc.robot.subsystems.RobotMap.*;
import frc.robot.controllers.*;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class RobotMech {

    public static final double MOTOR_POWER = 0.4;
    public static final double SHOOTER_POWER = -1.0;
    public static final double INTAKE_POWER = .5;
    public static final double ARM_MAX_LIMIT = 90;
    public static final double ARM_MIN_LIMIT = 0.0;

    public static final boolean CARGO_IN = false;
    public static final boolean CARGO_OUT = true;

    private RobotArm m_arm;
    private RobotRoller m_roller;
    private RobotShooter m_shooter;
    private SolenoidT m_hatchGrab;
    private DoubleSolenoid m_rollerSolenoids;
    private boolean m_noseState;
    private long m_lastNoseStateChange;
    private IntakeCargoFromFloor m_intakeCargoFromFloor;
    private AnalogPotentiometer m_armPotentiometer;

  //  public double armPos = m_armPotentiometer.get();


    public RobotMech() {
        /* Roller */
        try {
            m_roller = new RobotRoller(VictorPort.Roller);
        } catch (Exception e) {

            DriverStation.reportError("Couldn't instantiate Roller\n", false);
        }

        /* Arm */
        try {
            m_arm = new RobotArm(VictorPort.IntakeArm);
        } catch (Exception e) {
            DriverStation.reportError("Couldn't instantiate Arm\n", false);
        }

        /* Shooter */
        try {
            m_shooter = new RobotShooter(VictorPort.Shooter);
        } catch (Exception e) {
            DriverStation.reportError("Couldn't instantiate Shooter\n", false);
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

        /* Instantiate the Wrist Potentiometer */
        try {
            m_armPotentiometer = new AnalogPotentiometer(AnalogPort.ARM_POTENTIOMETER, 360, 0);

        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate Wrist Potentiometer\n", false);
        }

        /* Controllers */
        m_intakeCargoFromFloor = new IntakeCargoFromFloor(this, m_arm);
    }

    public void updateDashboard() {   
        SmartDashboard.putBoolean("Is Cargo Present?", m_arm.isCargoPresent());
        SmartDashboard.putBoolean("Is Arm At Upper Limit?", m_arm.isArmAtUpperLimit());
        SmartDashboard.putBoolean("Is Arm At Lower Limit?", m_arm.isArmAtLowerLimit());
      //  m_arm.updateDashboard();
    }

    public void updatePotentiometer() {   
        SmartDashboard.putNumber("Arm_Pos", m_armPotentiometer.get());
    }

    public void pushNoseOut()
    {
        if (!isNoseOut()) {
            m_rollerSolenoids.set(Value.kForward);
            m_lastNoseStateChange = System.currentTimeMillis();
            m_noseState = true;
        }
    }

    public void pullNoseIn()
    {
        if (isNoseOut()) {
            m_rollerSolenoids.set(Value.kReverse);
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

    public boolean isNoseActuallyOut()
    {
        // At some point, this will be replaced by a Limit Switch that will actually tell us the position of the Nose.
        // For now, though, we'll just see whether the last command was to push out, and whether sufficient time (1 second?) has passed.
        long minimum_time_since_nose_out = 1000;
        return (isNoseOut() && timeSinceLastNoseStateChange() >= minimum_time_since_nose_out);
    }

    public void periodic(Joystick stick) {
        if (stick == null) {
            DriverStation.reportError("No Joystick, cannot run Mech periodic\n", false);
            return;
        }

        // ARM : PWM 5
        // 7: Raise arm up
        // 9: Lower arm down

        if (stick.getRawButton(PlayerButton.MOVE_ARM_UP) && (!m_arm.isArmAtUpperLimit() || !(m_armPotentiometer.get() >= ARM_MIN_LIMIT))) {
                m_arm.moveArmUp();
            }
         else if (stick.getRawButton(PlayerButton.MOVE_ARM_DOWN) && (!m_arm.isArmAtLowerLimit() || !(m_armPotentiometer.get() >= ARM_MAX_LIMIT))) {
                m_arm.moveArmDown();
        } else {
            m_arm.stopArm();
        }


        // ROLLER : PWM 6
        // 3: Take cargo in
        // 5: Give ball out
        if (stick.getRawButton(PlayerButton.INTAKE_CARGO)) {
            m_intakeCargoFromFloor.do_intake();
        } else if (stick.getRawButton(PlayerButton.SPIT_OUT_CARGO)) {
            m_roller.pushRollerOut();
            m_shooter.stopShooter();
        } else if (stick.getRawButton(PlayerButton.FIRE_CARGO)) {
            m_shooter.fireShooter();
            m_roller.stopRoller();
        } else {
            m_shooter.stopShooter();
            m_roller.stopRoller();
        }


        // ROLLER_SOLENOIDS : PCM 3, 4
        // 11: solenoid toggle
        if (stick.getRawButton(PlayerButton.NOSE_IN)) {
            pullNoseIn();
        } else if (stick.getRawButton(PlayerButton.NOSE_OUT)) {
            pushNoseOut();
        }

        // HATCH_GRAB : PCM 5
        // Trigger: Unlock
        if (stick.getTrigger()) {
            m_hatchGrab.set(true);
        } else {
            m_hatchGrab.set(false);
        }

    }

    public void stopShooterRollers()
    {
        m_shooter.stopShooter();
    }

    public void stopIntakeRollers()
    {
        m_roller.stopRoller();
    }

    public void pullInShooterRollers()
    {
        m_shooter.intakeShooter();
    }

    public void pullInIntakeRollers()
    {
        m_roller.pullRollerIn();
    }

    private boolean enoughTimePassed(long timeNow, long initTime) {
        return timeNow > initTime + 1000;
    }

    public boolean isCargoPresent()
    {
        return m_arm.isCargoPresent();
    }

    long initTime = System.currentTimeMillis();
}
