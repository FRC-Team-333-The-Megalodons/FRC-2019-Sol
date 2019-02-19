package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Victor;

public class RobotShooter {

    public static final int INTAKE_BUTTON = 3;
    public static final int SHOOTER_BUTTON = 4;
    public static final double SHOOTER_POWER = -1.0;
    public static final double INTAKE_POWER = .5;
    public static final boolean CARGO_OUT = true;

    private CANSparkMax m_topSpark, m_bottomSpark;

    public RobotShooter(int topDeviceID, MotorType topType, int bottomDeviceID, MotorType bottomType) {
        /* Instantiate the Shooter */
        try {
            m_topSpark = new CANSparkMax(topDeviceID, topType);
            m_bottomSpark = new CANSparkMax(bottomDeviceID, bottomType);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Shooter\n", false);
        }
    }

    public void fireShooter()
    {
        m_topSpark.set(SHOOTER_POWER);
        m_bottomSpark.set(SHOOTER_POWER);
    }

    public void intakeShooter()
    {
        m_topSpark.set(INTAKE_POWER);
        m_bottomSpark.set(INTAKE_POWER);
    }

    public void stopShooter()
    {
        m_topSpark.set(0.0);
        m_bottomSpark.set(0.0);
    }
}