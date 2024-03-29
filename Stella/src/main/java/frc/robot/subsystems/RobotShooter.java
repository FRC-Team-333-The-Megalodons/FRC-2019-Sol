package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotShooter {

    public static final int INTAKE_BUTTON = 3;
    public static final int SHOOTER_BUTTON = 4;
    public static final double FULL_SHOOTER_POWER = 0.8;
    public static final double ROCKET_SHOOTER_POWER = 0.75;
    public static final double DROOL_TOP_POWER = 0.4;
    public static final double DROOL_BOTTOM_POWER = 0.4;
    public static final double INTAKE_POWER = -0.5;
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

    public void droolShot()
    {
        droolShot(DROOL_TOP_POWER, DROOL_BOTTOM_POWER);
    }

    private void droolShot(double topPower, double bottomPower)
    {
        m_topSpark.set(topPower);
        m_bottomSpark.set(bottomPower);
    }

    public void fireShooter(double power)
    {
        m_topSpark.set(power);
        m_bottomSpark.set(power);
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