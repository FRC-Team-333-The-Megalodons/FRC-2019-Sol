package frc.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotArm {

    public static final int ARM_UP_BUTTON    = 7;
    public static final int ARM_DOWN_BUTTON  = 9;

    public RobotArm(int port) {
        /* Instantiate the Arm */
        try {
            m_talon = new Talon(port);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Arm\n", false);
        }
    }

    public void moveArmUp()
    {
        m_talon.set(.75);
    }

    public void moveArmDown()
    {
        m_talon.set(-.10);
    }

    public void stopArm()
    {
        m_talon.set(0);
    }
        
    private Talon m_talon;
}


class RobotArmState {

    public static final boolean ARM_UP      = true;
    public static final boolean ARM_DOWN    = false;

    public RobotArmState(int port) {

        try {
            m_limitSwitch = new DigitalInput(port);
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
