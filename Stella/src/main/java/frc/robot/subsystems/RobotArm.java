package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.*;
import frc.robot.subsystems.RobotMap.*;

public class RobotArm {

    public static final int ARM_UP_BUTTON    = 7;
    public static final int ARM_DOWN_BUTTON  = 9;

    private RobotCargoState m_cargoState;
    private RobotArmState m_upperArmLimit, m_lowerArmLimit;
   // private Gyro m_gyro;
    private Victor m_victor;

    public RobotArm(int port) {
        /* Instantiate the Arm */
        try {
            m_victor = new Victor(port);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Arm\n", false);
        }

    /*    try {
            m_gyro = new AnalogGyro(0);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Arm Gyro\n", false);
        }
      */  
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
    }

    public boolean isCargoPresent()
    {
        return m_cargoState.isCargoPresent();
    }

    public boolean isArmAtUpperLimit()
    {
        return m_upperArmLimit.isArmAtLimit();
    }

    public boolean isArmAtLowerLimit()
    {
        return m_lowerArmLimit.isArmAtLimit();
    }
/*
    public void updateDashboard()
    {
        SmartDashboard.putNumber("Gyro Reading", m_gyro.getAngle());
    }

    public boolean isArmAtHigh()
    {
        return m_gyro.getAngle() >= 90.0;
    }

    public boolean isArmAtMiddle()
    {
        return m_gyro.getAngle() >= 50.0;
    }
*/
    public boolean isArmAtLow()
    {
        return isArmAtLowerLimit();
    }

    public void set(double speed)
    {
        m_victor.set(speed);
    }

    public void moveArmUp()
    {
        m_victor.set(.75);
    }

    public void moveArmDown()
    {
        m_victor.set(-.30);
    }

    public void holdArmAtHigh()
    {
        double power = SmartDashboard.getNumber("ArmAtHighMotorStallPower",0.0f);
        m_victor.set(power);
    }

    public void holdArmAtMiddle()
    {
        double power = SmartDashboard.getNumber("ArmAtMiddleMotorStallPower", 0.0f);
        m_victor.set(power);
    }

    public void stopArm()
    {
        m_victor.set(0);
    }
        
}

class RobotArmPos {

    public static final double ARM_MAX      = 47.0;
    public static final double ARM_MIN      = 40.0;
    public static final double CLOSE_TO_MAX = 45.0;
    public static final double CLOSE_TO_MIN = 42.0;

    public RobotArmPos(int port) {

        try {
            m_potentiometer = new AnalogPotentiometer(port);
        } catch (Exception e) {
         DriverStation.reportError("Could not instantiate Potentiometer\n", false);
        }
    }

    public double get()
    {
        return m_potentiometer.get();
    }

    public boolean isCloseToMaxPotValue()
    {
        return (m_potentiometer.get() == CLOSE_TO_MAX);
    }

    public boolean isCloseToMinPotValue()
    {
        return (m_potentiometer.get() == CLOSE_TO_MIN);
    }

    public boolean isArmAtMaxPotValue()
    {
        return (m_potentiometer.get() == ARM_MAX);
    }

    public boolean isArmAtMinPotValue()
    {
        return (m_potentiometer.get() == ARM_MIN);
    }

    private AnalogPotentiometer m_potentiometer;
}


class RobotArmState {

    public static final boolean ARM_UP      = true;
    public static final boolean ARM_DOWN    = false;


    public RobotArmState(int limit_port) {

        try {
            m_limitSwitch = new DigitalInput(limit_port);
       } catch (Exception e) {
           DriverStation.reportError("Could not instantiate limit switch\n", false);
       }



       SmartDashboard.putBoolean("Arm Limit", m_limitSwitch.get());

    }

    public boolean isArmAtLimit()
    {
        return (m_limitSwitch.get() == ARM_UP);
    }

    private DigitalInput m_limitSwitch;
}


class RobotCargoState {

    public static final boolean CARGO_IN     = false;
    public static final boolean CARGO_OUT    = true;

    public RobotCargoState(int port) {

        try {
            m_limitSwitch = new DigitalInput(port);
       } catch (Exception e) {
           DriverStation.reportError("Could not instantiate limit switch\n", false);
       }

       SmartDashboard.putBoolean("Limit", m_limitSwitch.get());

    }

    public boolean isCargoPresent()
    {
        return (m_limitSwitch.get() == CARGO_IN);
    }

    private DigitalInput m_limitSwitch;
}
