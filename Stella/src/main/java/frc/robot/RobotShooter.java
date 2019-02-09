package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

public class RobotShooter {

    public static final int INTAKE_BUTTON = 3;
    public static final int SHOOTER_BUTTON = 4;
    public static final double SHOOTER_POWER = -1.0;
    public static final double INTAKE_POWER = .5;
    public static final boolean CARGO_IN = false;
    public static final boolean CARGO_OUT = true;

    private Talon m_talon;
    private RobotCargoState m_cargoState;

    public RobotShooter(int port) {
        /* Instantiate the Shooter */
        try {
            m_talon = new Talon(port);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Shooter\n", false);
        }
    }

    public void fireShooter()
    {
        m_talon.set(SHOOTER_POWER);
    }

    public void intakeShooter()
    {
        m_talon.set(INTAKE_POWER);
    }

    public void stopShooter()
    {
        m_talon.set(0.0);
    }
}