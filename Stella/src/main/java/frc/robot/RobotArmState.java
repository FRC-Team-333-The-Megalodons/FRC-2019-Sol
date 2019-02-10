package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotArmState {

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
