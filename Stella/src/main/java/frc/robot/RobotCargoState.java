package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotCargoState {

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
