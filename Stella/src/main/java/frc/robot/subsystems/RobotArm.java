package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.subsystems.RobotMap.*;

public class RobotArm {

    private RobotCargoState m_cargoState;
    private DigitalInput m_upperArmLimit, m_lowerArmLimit;
   // private Gyro m_gyro;
    private CANSparkMax m_armNeo;

    private RobotArmPos m_position;
    public static final double SHOOTING_POSITION     = RobotMap.RobotType.isFinal ? 100.0 : -109.0;
    public static final double TOP_POSITION          = RobotMap.RobotType.isFinal ? 130.0 : -131.0;
    public static final double CARGO_TRAVEL_POSITION = RobotMap.RobotType.isFinal ? 50.0 : -40.0;
    public static final double BOTTOM_POSITION       = RobotMap.RobotType.isFinal ? 0.0 : 0.0;
    public static final double ROCKET_SHOOTING_POS   = RobotMap.RobotType.isFinal ? 70.50 : -69.5;
    private final double UPWARD_MOTOR_POWER          = RobotMap.RobotType.isFinal ? 1.0 : -1.0;
    private final double DOWNWARD_MOTOR_POWER        = RobotMap.RobotType.isFinal ? -1.0 : 1.0;
    private final double UPPER_LIMIT_SWITCH_POS      = RobotMap.RobotType.isFinal ? 150.0 : -137.0;
    private final double LOWER_LIMIT_SWITCH_POS      = RobotMap.RobotType.isFinal ? 0.0 : 0.0;
    private final double ARM_POS_TOLERANCE      = 0.5;
    private final double ARM_CLOSE_THRESHOLD    = 15.0;
    private final double ARM_MOTOR_MINIMUM_FACTOR = 0.2;

    public RobotArm(int port) {

        /* Instantiate the Arm */
        try {
            m_armNeo = new CANSparkMax(port, MotorType.kBrushless);
            m_position = new RobotArmPos(m_armNeo.getEncoder());
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Arm\n", false);
        }

        try {
            m_cargoState = new RobotCargoState(DigitalInputPort.CLAW_SWITCH);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate claw limit switch\n", false);
        }

        try {
            m_upperArmLimit = new DigitalInput(DigitalInputPort.UPPER_ARM_SWITCH);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate upper arm limit switch\n", false);
        }

        try {
            m_lowerArmLimit = new DigitalInput(DigitalInputPort.LOWER_ARM_SWITCH);
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate lower arm limit switch\n", false);
        }
    }

    public boolean isCargoPresent()
    {
        return m_cargoState.isCargoPresent();
    }

    public boolean isArmAtUpperLimit()
    {
        return m_upperArmLimit.get();
    }

    public boolean isArmAtLowerLimit()
    {
        return m_lowerArmLimit.get();
    }

    public void updateDashboard()
    {
        SmartDashboard.putBoolean("Is Cargo Present?", isCargoPresent());
        SmartDashboard.putBoolean("Is Arm At Lower Limit?", isArmAtLowerLimit());
        SmartDashboard.putBoolean("Is Arm At Upper Limit?", isArmAtUpperLimit());
        SmartDashboard.putNumber("Arm Hall Encoder (raw):", m_position.getRawPosition());
        SmartDashboard.putNumber("Arm Hall Encoder (scaled):", m_position.get());
        SmartDashboard.putNumber("Arm Hall Encoder (offset):", m_position.getOffset());
    }

    public void moveArmUp(double factor)
    {
        if (isArmAtUpperLimit()) {
            stopArm();
            return;
        }

        factor = Math.max(factor, ARM_MOTOR_MINIMUM_FACTOR);

        m_armNeo.set(UPWARD_MOTOR_POWER * factor);
    }

    public void moveArmDown(double factor)
    {
        if (isArmAtLowerLimit()) {
            stopArm();
            return;
        }

        factor = Math.max(factor, ARM_MOTOR_MINIMUM_FACTOR);
        
        m_armNeo.set(DOWNWARD_MOTOR_POWER*factor);
    }

    public void tareArmEncoderIfNeeded()
    {
        if (isArmAtUpperLimit()) {
            m_position.tare(UPPER_LIMIT_SWITCH_POS);
        } else if (isArmAtLowerLimit()) {
            m_position.tare(LOWER_LIMIT_SWITCH_POS);
        }
    }

    public void stopArm()
    {
        m_armNeo.set(0.0);
    }

    public boolean isArmAtTarget(double target)
    {
        return isArmAtTarget(target, 0.0);
    }

    public boolean isArmAtTarget(double target, double addtl_tolerance)
    {        
        double current = m_position.pidGet();

        double gap = Math.abs(current - target);

        return (gap <= (addtl_tolerance+ARM_POS_TOLERANCE));
    }

    public boolean periodic(Double target_obj)
    {
        tareArmEncoderIfNeeded();

        if (target_obj == null) {
            stopArm();
            return true;
        }

        double target = target_obj.doubleValue();

        double current = m_position.pidGet();

        double gap = Math.abs(current - target);

        if (isArmAtTarget(target)) {
            stopArm();
            return true;
        }

        double factor = 1.0;
        if (gap <= ARM_CLOSE_THRESHOLD) {
            factor = (gap / ARM_CLOSE_THRESHOLD);
        } 

        if (RobotMap.RobotType.isFinal ? current < target : target < current) {
            moveArmUp(factor);   
        } else {
            moveArmDown(factor);
        }
        return false;
    }

    public RobotCargoState getCargoState()
    {
        return m_cargoState;
    }
}

class RobotArmPos implements PIDSource {
    private CANEncoder m_encoder;
    private double m_offset = 0.0;
    private PIDSourceType m_pidSourceType = PIDSourceType.kDisplacement;

    public RobotArmPos(CANEncoder encoder) {
        m_encoder = encoder;
    }

    public void tare(double expected)
    {
        m_offset = expected - m_encoder.getPosition();
    }

    public PIDSourceType getPIDSourceType()
    {
        return m_pidSourceType;
    }

    public void setPIDSourceType(PIDSourceType type)
    {
        m_pidSourceType = type;
    }

    public double pidGet()
    {
        if (m_pidSourceType == PIDSourceType.kDisplacement) {
            return m_encoder.getPosition() + m_offset;
        } else {
            return m_encoder.getVelocity();
        }
    }

    public double get()
    {
        return pidGet();
    }

    public double getRawPosition()
    {
        return m_encoder.getPosition();
    }

    public double getOffset()
    {
        return m_offset;
    }
}

class RobotCargoState {

    public static final boolean CARGO_IN     = RobotMap.RobotType.isFinal ? true : false;
    public static final boolean CARGO_OUT    = RobotMap.RobotType.isFinal ? false : true;

    public RobotCargoState(int port) {

        try {
            m_limitSwitch = new DigitalInput(port);
       } catch (Exception e) {
           DriverStation.reportError("Could not instantiate limit switch\n", false);
       }
    }

    public boolean isCargoPresent()
    {
        return (m_limitSwitch.get() == CARGO_IN);
    }

    private DigitalInput m_limitSwitch;
}
