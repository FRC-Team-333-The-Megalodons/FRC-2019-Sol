package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotRoller{

    private CANSparkMax m_motor;

    public RobotRoller(int port) {
        /* Instantiate the Roller */
        try {
            m_motor = new CANSparkMax(port, MotorType.kBrushed);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Roller\n", false);
        }
    }

    public void pushRollerOut()
    {
        m_motor.set(-1);
    }

    public void pullRollerIn()
    {
        m_motor.set(1);
    }

    public void stopRoller()
    {
        m_motor.set(0);
    }
}