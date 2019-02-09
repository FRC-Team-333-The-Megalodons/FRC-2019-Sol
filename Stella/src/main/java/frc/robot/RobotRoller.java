package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Talon;

public class RobotRoller{

    private Talon m_talon;

    public RobotRoller(int port) {
        /* Instantiate the Roller */
        try {
            m_talon = new Talon(port);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Roller\n", false);
        }
    }

    public void pushRollerOut()
    {
        m_talon.set(-1);
    }

    public void pullRollerIn()
    {
        m_talon.set(1);
    }

    public void stopRoller()
    {
        m_talon.set(0);
    }
}
