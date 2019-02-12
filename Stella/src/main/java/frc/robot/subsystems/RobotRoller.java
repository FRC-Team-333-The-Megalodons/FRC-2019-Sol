package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Victor;

public class RobotRoller{

    private Victor m_victor;

    public RobotRoller(int port) {
        /* Instantiate the Roller */
        try {
            m_victor = new Victor(port);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Roller\n", false);
        }
    }

    public void pushRollerOut()
    {
        m_victor.set(-1);
    }

    public void pullRollerIn()
    {
        m_victor.set(1);
    }

    public void stopRoller()
    {
        m_victor.set(0);
    }
}
