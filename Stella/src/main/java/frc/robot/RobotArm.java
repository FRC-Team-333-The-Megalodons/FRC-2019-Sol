package frc.robot;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DriverStation;

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