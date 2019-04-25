package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RobotMap.LimelightLEDMode;
import frc.robot.subsystems.RobotMap.LimelightType;
import frc.robot.subsystems.RobotMap.PlayerButton;
import frc.robot.subsystems.RobotMap.RobotType;

public class RobotUtils {

    public static boolean dbl_equals_power(double a, double b, int precision) {
        return Math.abs(a - b) <= Math.pow(10, -precision);
    }

    public static boolean dbl_equals(double a, double b, double precision) {
        return Math.abs(a - b) <= precision;
    }

    public static double abs_min(double a, double abs_b) {
        double sign = (a < 0 ? -1 : 1);
        return Math.min(Math.abs(a), abs_b) * sign;
    }

    public static void updateLimelightPipeline(NetworkTableEntry pipeline, Number pipeline_index)
    {
        int current_pipeline_index = pipeline.getNumber(-1).intValue();
        int new_pipeline_index = pipeline_index.intValue();
        if (new_pipeline_index != current_pipeline_index) {
            System.out.println("Setting Limelight Pipeline from "+current_pipeline_index+" to "+new_pipeline_index);
            pipeline.setNumber(new_pipeline_index);
        }
    }
}

class SolenoidT {
    private Solenoid m_solenoid;
    private Long m_lastFalse, m_lastTrue;

    public SolenoidT(int id) {
        m_solenoid = new Solenoid(id);
    }

    public void set(boolean value) {
        boolean last = m_solenoid.get();
        m_solenoid.set(value);
        if (last != value) {
            long now = System.currentTimeMillis();
            if (value) {
                m_lastTrue = now;
            } else {
                m_lastFalse = now;
            }
        }
    }

    public Long getLastBecameTrue() {
        return m_lastTrue;
    }

    public Long getLastBecameFalse() {
        return m_lastFalse;
    }
}

class Solenoidal {
    // Solenoids always come in pairs that are inverses of each other.
    // A "Solenoidal" is a wrapper around the solenoids to represent the two as
    // a single entity.
    private Solenoid m_solenoid1, m_solenoid2;
    private boolean m_state;

    public Solenoidal(int id1, int id2) {
        m_solenoid1 = new Solenoid(id1);
        m_solenoid2 = new Solenoid(id2);
    }

    public void set(boolean value) {
        m_state = value;
        m_solenoid1.set(m_state);
        m_solenoid2.set(!m_state);
    }

    public boolean get() {
        return m_state;
    }
}

class SolenoidalPair {
    private Solenoidal m_solenoidal1, m_solenoidal2;

    public SolenoidalPair(Solenoidal sol1, Solenoidal sol2) {
        m_solenoidal1 = sol1;
        m_solenoidal2 = sol2;
    }

    public void set(boolean value) {
        m_solenoidal1.set(value);
        m_solenoidal2.set(value);
    }
}

class VictorPair implements PIDOutput {
    private Victor m_motor1, m_motor2;
    private double m_sign;

    public VictorPair(int id1, int id2, boolean inverted) {
        m_motor1 = new Victor(id1);
        m_motor2 = new Victor(id2);
        m_sign = inverted ? -1.0 : 1.0;
    }

    public void set(double value) {
        m_motor1.set(value);
        m_motor2.set(-value * m_sign);
    }

    @Override
    public void pidWrite(double output) {
        m_motor1.pidWrite(output);
        m_motor2.pidWrite(-output * m_sign);
    }
}

class UltrasonicMonitor implements Runnable {
    Ultrasonic m_ultrasonic;

    public UltrasonicMonitor(Ultrasonic ultrasonic) {
        m_ultrasonic = ultrasonic;
    }

    @Override
    public void run() {
        while (m_ultrasonic != null) {
            SmartDashboard.putNumber("Ultrasonic", m_ultrasonic.getRangeInches());
            Timer.delay(0.05);
        }
    }
}

class ArmPotentiometerMonitor implements Runnable {
    AnalogPotentiometer m_armPotentiometer;

    public ArmPotentiometerMonitor(AnalogPotentiometer potentiometer) {
        m_armPotentiometer = potentiometer;
    }

    @Override
    public void run() {
        while (m_armPotentiometer != null) {
            SmartDashboard.putNumber("Arm Potentiometer", m_armPotentiometer.get());
            Timer.delay(0.05);
        }
    }
}

class LimitSwitchMonitor implements Runnable {
    DigitalInput m_limitSwitch;

    public LimitSwitchMonitor(DigitalInput limitSwitch) {
        m_limitSwitch = limitSwitch;
    }

    @Override
    public void run() {
        while (m_limitSwitch != null) {
            SmartDashboard.putBoolean("Limit Switch Value", m_limitSwitch.get());
            Timer.delay(0.05);
        }
    }
}

class DummyAHRS {
    public DummyAHRS(Port port) {
    }

    public void reset() {
        appendToHistory(0.0);
        averageHistory();
    }

    public void resetDisplacement() {
    }

    public double getAngle() {
        return 123.45;
    }

    private double averageHistory() {
        double sum = 0.0;
        for (int i = 0; i < m_transHistory.size(); i++) {
            sum += m_transHistory.get(i);
        }
        return sum / m_transHistory.size();
    }

    private void appendToHistory(double data) {
        m_transHistory.add(data);
        if (m_transHistory.size() > HISTORY_LIMIT) {
            m_transHistory.remove(0);
        }
    }

    private ArrayList<Double> m_transHistory;
    private static double HISTORY_LIMIT = 20;
    public static double MAX_RANGE = 4800.0;
}

class MultiLimitSwitch {
    private DigitalInput[] m_limitSwitches;

    public MultiLimitSwitch(DigitalInput... limitSwitches) {
        m_limitSwitches = limitSwitches;
    }

    public boolean get()
    {
        for (DigitalInput limitSwitch : m_limitSwitches) {
            if (limitSwitch != null && !limitSwitch.get()) {
                return false;
            }
        }
        return true;
    }
}

class MultiSpeedController implements SpeedController {

    private SpeedController[] m_speedControllers;
    private double m_speed;
    private boolean m_inverted;

    public MultiSpeedController(SpeedController... speedControllers) {
        m_speedControllers = speedControllers;
        set(0);
    }

    @Override
    public double get() {
        return m_speed;
    }

    @Override
    public void set(double speed) {
        m_speed = speed;

        for (SpeedController speedController : m_speedControllers) {
            speedController.set(m_speed);
        }
    }

    @Override
    public void pidWrite(double output) {
        set(output);
    }

    @Override
    public void disable() {
        for (SpeedController speedController : m_speedControllers) {
            speedController.disable();
        }
    }

    @Override
    public boolean getInverted() {
        return m_inverted;
    }

    @Override
    public void setInverted(boolean inverted) {
        m_inverted = inverted;
        for (SpeedController speedController : m_speedControllers) {
            speedController.setInverted(m_inverted);
        }
    }

    @Override
    public void stopMotor() {
        m_speed = 0.0;

        for (SpeedController speedController : m_speedControllers) {
            speedController.stopMotor();
        }
    }
}

class HistoryWrapper
{
    private History m_history = null;

    HistoryWrapper(History history)
    {
        m_history = history;
    }

    double getHistoryAverage(double value)
    {
        if (m_history == null) {
            return value;
        }
        m_history.appendToHistory(value);
        return m_history.getHistoryAverage();
    }

    double getHistoryAverageWithSalt(double value) {
        if (m_history == null) {
            return value;
        }
        return m_history.getHistoryAverageWithSalt(value);
    }
}

// This class is a significantly simplified version of a PID controller,
// that re-evalutes only at the time of "setSetpoint" rather than in a
// background thread.
class SimplePIDController {

    private double m_abs_output_min, m_abs_output_max, m_threshold_delta, m_tolerance, m_speedModifier;
    private Double m_target;
    private PIDSource m_source;
    private PIDOutput m_output;
    private DigitalInput m_lowerLimitSwitch, m_upperLimitSwitch;
    private HistoryWrapper m_transHistory;

    // Assume that min & max are absolute values, and can be negative on our real
    // output device.
    public SimplePIDController(PIDSource source, PIDOutput output, 
                               DigitalInput lowerLimitSwitch,
                               DigitalInput upperLimitSwitch, // Limit Switches (can be null)
                               double abs_output_min, double abs_output_max, // Output Min/Max
                               double threshold_delta, double tolerance, int history_len_ms) {

        History history = null;
        if (history_len_ms > 0) {
            history = new History(history_len_ms, false);
        }
        m_transHistory = new HistoryWrapper(history);
        m_abs_output_min = abs_output_min;
        m_abs_output_max = abs_output_max;
        m_source = source;
        m_output = output;
        m_threshold_delta = threshold_delta;
        m_tolerance = tolerance;
        m_lowerLimitSwitch = lowerLimitSwitch;
        m_speedModifier = 1.0;
    }

    public double getCurrent() {
        return m_source.pidGet();
    }

    public boolean hasArrived(double target) {
        double current = m_transHistory.getHistoryAverageWithSalt(m_source.pidGet());
        return Math.abs(current - target) <= m_tolerance;
    }

    public boolean isProposedSetpointAbove(double target) {
        double current = m_transHistory.getHistoryAverageWithSalt(m_source.pidGet());
        return current < target;
    }

    public void setSetpoint(Double target) {
        m_target = target;
        periodic();
    }

    public void stop() {
        m_output.pidWrite(0.0);
    }

    public void setSpeedModifier(double mod) {
        m_speedModifier = mod;
    }

    public void resetSpeedModifier() {
        m_speedModifier = 1.0;
    }

    private void periodic() {
        if (m_source.getPIDSourceType() != PIDSourceType.kDisplacement) {
            System.out.println("Invalid PID Source Type");
            return;
        }
        if (m_target == null) {
            m_output.pidWrite(0.0);
            return;
        }

        double current = m_transHistory.getHistoryAverage(m_source.pidGet());
        double target = m_target.doubleValue();
        double delta = Math.abs(current - target);
        if (delta <= m_tolerance) {
            m_output.pidWrite(0.0);
            return;
        }
        double sign = (current < target ? -1.0 : 1.0);
        // Check the Limit Switch, just in case!
        DigitalInput limitSwitch = (sign < 0 ? m_upperLimitSwitch : m_lowerLimitSwitch);
        if (limitSwitch != null && limitSwitch.get()) {
            m_output.pidWrite(0.0);
            return;
        }
        double output = sign * m_abs_output_max;
        if (delta < m_threshold_delta) {
            double ratio = delta / m_threshold_delta;
            double range = m_abs_output_max - m_abs_output_min;
            output = sign * ((ratio * range) + m_abs_output_min);
        }
        output *= m_speedModifier;

        System.out.println("Target="+target+"; Current="+current+"; Planned Output="+output);

        // System.out.println("PIDWRITE "+output+" (sign=" + sign +", cap="+
        // m_abs_output_max+")");
        m_output.pidWrite(output);
    }
}

class DriveTrainMotor implements PIDSource, PIDOutput
{
    private CANSparkMax m_motor;
    private CANEncoder m_encoder;
    private PIDSourceType m_pidType;

    public DriveTrainMotor(CANSparkMax motor)
    {
        m_motor = motor;
        m_encoder = m_motor.getEncoder();
        m_pidType = PIDSourceType.kDisplacement;
    }

    // The real PID routines!
    @Override
    public double pidGet() {
        switch (m_pidType)  {
            case kDisplacement:
                return m_encoder.getPosition();
            case kRate:
                return m_encoder.getVelocity();
            default:
                return -1.0;
        }
    }

    @Override
    public void pidWrite(double output) {
        m_motor.set(-output);
    }

    // Other routines we don't use, but are technically required by the interface
    @Override
    public void setPIDSourceType(PIDSourceType pidType) {
        m_pidType = pidType;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return m_pidType;
    }
}

// for Cargo Hatch:
//   - Left cap at 0.7, Right cap at 0.6
//   - Left = -56.3, right = 53.6

// for Rocket hatch
//   - Left cap at 0.8, Right cap at 0.6
//   - Left = -72.57, right = 66.52

class AutonArcDrive {
    private double m_leftTarget, m_rightTarget;
    SimplePIDController m_leftController, m_rightController;
    private DriveTrainMotor m_leftMotor, m_rightMotor;

    public AutonArcDrive(CANSparkMax leftMotor, CANSparkMax rightMotor,
                      double leftTarget, double rightTarget,
                      double leftCap, double rightCap)
    {                
        m_leftMotor = new DriveTrainMotor(leftMotor);
        m_rightMotor = new DriveTrainMotor(rightMotor);
        m_leftTarget = leftTarget;
        m_rightTarget = rightTarget;

        m_leftController = new SimplePIDController(m_leftMotor, m_leftMotor, null, null, 0.0, leftCap, 20.0, 2.0, 0);
        m_rightController = new SimplePIDController(m_rightMotor, m_rightMotor, null, null, 0.0, rightCap, 20.0, 2.0, 0);

        // At this point, we can figure out a ratio that we can apply to set a multiplier to get the arc that we'd like.


    }

    public boolean periodic()
    {
        boolean arrived = m_leftController.hasArrived(m_leftTarget) &&
                          m_rightController.hasArrived(m_rightTarget);
        if (arrived) {
            stop();
            return true;
        }

        m_leftController.setSetpoint(m_leftTarget);
        m_rightController.setSetpoint(m_rightTarget);
        return false;
    }

    public void stop()
    {
        m_leftController.stop();
        m_rightController.stop();
    }

}


// This is wrapper class around RobotDrive that acts as an automatic
// transmission.
// It sets thresholds beyond which it'll shift.
// The logic hierarchy is as follows:
// 1) if we're powering at below LOW_THRESHOLD, always shift into low gear.
// 2) if we're powering above HIGH_THRESHOLD, and we have been for the last
// half-second, switch into HIGH.
class TeleopTransDrive {
    public static final int TRANS_HISTORY_LEN_MS = 250;
    public static final int SPEED_HISTORY_LEN_MS = 1000;
    public static final double HIGH_THRESHOLD = 0.80;
    public static final double LOW_THRESHOLD = 0.40;
    public static final double SLOW_MODE_CAP = 0.70;

    private int m_button_forceLowTrans;
    private boolean m_forceHighTrans;
    private History m_transHistory;
    private DifferentialDrive m_drive;
    private Solenoidal m_transmission;

    public TeleopTransDrive(DifferentialDrive drive, Solenoidal transmission, int button_forceLowTrans, boolean forceHighTrans) {
        m_transmission = transmission;
        m_drive = drive;
        m_drive.setExpiration(0.1);
        m_drive.setSafetyEnabled(true);
        m_drive.setMaxOutput(1.0);
        m_button_forceLowTrans = button_forceLowTrans;
        m_forceHighTrans = forceHighTrans;
        m_transHistory = new History(TRANS_HISTORY_LEN_MS, true);
    }

    public void drive(Joystick stick, boolean curvature, Double abs_limit) {

        double joystick_Y = stick.getY();
        double joystick_X = stick.getX();

        // If the elevator is up, just limit our inputs.
        if (abs_limit != null) {
            joystick_Y = RobotUtils.abs_min(joystick_Y, abs_limit.doubleValue());
            joystick_X = RobotUtils.abs_min(joystick_X, abs_limit.doubleValue());
        }

        boolean forceLowTrans = stick.getRawButton(m_button_forceLowTrans);

        // Actually do the Drive
        if (curvature && !forceLowTrans) {
            boolean isQuickTurn = Math.abs(stick.getX()) > 0.9  //if the joystick x (sideways) value is above this pivot turns are possible as long as the driver isn't going forward
                                  && Math.abs(stick.getY()) < 0.2; //controls how much the driver needs to go forward before curvature kicks in
            double power = Math.pow(joystick_Y, 3);
            double rotation = -joystick_X;
            m_drive.curvatureDrive(power, rotation, isQuickTurn);
        } else {
            // We're running into an issue where the turning with Neos snaps too quickly. 
            // TODO: Make this more quadratic.
            //   For now, just taper off when joystick_X is below 0.95.

            double REDUCTION_THRESHOLD_X = 1.0;
            double REDUCTION_FACTOR_X = 0.75;
            if (Math.abs(joystick_X) <= REDUCTION_THRESHOLD_X) {
                joystick_X *= REDUCTION_FACTOR_X;
            }

            m_drive.arcadeDrive(joystick_Y, -joystick_X);
        }


        // If forceHighTrans was specified
        if (m_forceHighTrans && !forceLowTrans) {
            if (highTransmission()) {
                System.out.println("Auto-Switching to High Transmission");
            }
            return;
        }

        // Enter this section only if we're actually doing auto-shifting.

        // If Auto shifting is on, we should be keeping track of the history, low or high gear.
        m_transHistory.appendToHistory(joystick_Y);

        // If the driver is holding the Low Transmission button, force it into low transmission.
        if (forceLowTrans) {
            if (lowTransmission()) {
                System.out.println("Force-Switching to Low Transmission");
            }
            return;
        }

        // If we've made it this far, we need to make a decision about 
        //   shifting given the historical joystick averages!
        double transHistoryAverage = m_transHistory.getHistoryAverage();

        // If the driver is holding the joystick below the Low Transmission
        //   threshold for long enough, downshift into Low Gear.
        if (transHistoryAverage < LOW_THRESHOLD) {
            if (lowTransmission()) {
                System.out.println("Auto-Switching to Low Transmission");
            }
            return;
        }
        // If the driver is holding the joystick above the High Transmission 
        //   threshold for long enough, upshift into High Gear.
        if (transHistoryAverage >= HIGH_THRESHOLD) {
            if (highTransmission()) {
                System.out.println("Auto-Switching to High Transmission");
            }
            return;
        }
    }

    public void arcadeDrive(Joystick stick, Double abs_limit)
    {
      drive(stick, false, abs_limit);
    }

    public void curvatureDrive(Joystick stick, Double abs_limit) {
      drive(stick, true, abs_limit);
    }

    public void arcadeDrive(double speed, double angle) {
        highTransmission();
        if (m_drive != null) {
            if (speed == 0.0 && angle == 0.0) {
                m_drive.stopMotor();
            } else {
                m_drive.arcadeDrive(speed, angle);
            }
        }
    }

    public boolean setTransmission(boolean target) {
        if (m_transmission == null) {
            DriverStation.reportError("Transmission called, but not instantiated\n", false);
            return false;
        }

        boolean changed = m_transmission.get() != target;
        m_transmission.set(target);
        return changed;
    }

    public boolean lowTransmission() {
        return setTransmission(false);
    }

    public boolean highTransmission() {
        return setTransmission(true);
    }
}

class LimelightDrive {
    public static final double kAIM = 0.45;
    public static final double kDistance = 5.125;
    public static final double kMinInc = 0.05;
    public static final double HATCH_PICKUP_DISTANCE = (RobotType.isFinal ? 6.05 : 5.50); //TODO: Change zeros into actual values.
    public static final double CARGO_SHOOT_DISTANCE = LimelightType.isOriginal ? 5.0 : 0.0;
    public static final double ROCKET_SHOOT_DISTANCE = LimelightType.isOriginal ? 4.5 : 0.0; 
    
    private DifferentialDrive m_drive;
    private Solenoidal m_transmission;
    private RobotCargoState m_cargoState;

    //private RobotHatchGrab m_hatchGrab;

    public LimelightDrive(DifferentialDrive drive, Solenoidal transmission, RobotHatchGrab hatchGrab, RobotCargoState cargoState) {
        m_transmission = transmission;
        m_drive = drive;
        //m_hatchGrab = hatchGrab;
        m_cargoState = cargoState;

        //m_drive.setExpiration(0.1);
        //m_drive.setSafetyEnabled(true);
        //m_drive.setMaxOutput(0.5);
    }

    public void autoDrive(int limelightPipeline, double tx, double ty, double area, boolean rocketMode) {

        // Initialize configurations to RobotMap.LimelightPipeline.HATCH values.
        double KpAim = kAIM; //SmartDashboard.getNumber("AutoDrive_kAIM", kAIM);
        double KpDistance = kDistance; //SmartDashboard.getNumber("AutoDri ve_kDistance", kDistance);
        double min_aim_command = kMinInc; //SmartDashboard.getNumber("AutoDrive_minInc", 0.05f);
        double max_area = HATCH_PICKUP_DISTANCE;
        if (m_cargoState.isCargoPresent()) {
            max_area = CARGO_SHOOT_DISTANCE;
            if (rocketMode) {
              max_area = ROCKET_SHOOT_DISTANCE;
            }
        }
        double min_area = 0.0;

        /*
        // If we're actually in CARGO hunting mode, then tweak our values.
        if (limelightPipeline == RobotMap.LimelightPipeline.CARGO) {

        }
        */

        
        double max_x = 23;
        double min_x = -23;

        // Flip the area; it's inverted (bigger is target originally);
        if (area != 0.0f) {
            area = max_area-area;
        }

        double max_range = (max_x + max_area);
        double min_range = (min_x + min_area);

        double range = (max_range - min_range) / 2; // 25.5
        double offset = (max_range - range); // 2.5

        double heading_error = -tx;


        // flip the area

        double distance_error = -area; //-ty;
        double steering_adjust = 0.0f;

        if (tx > 0.0) {
            steering_adjust = KpAim * heading_error - min_aim_command;
        } else if (tx < 0.0) {
            steering_adjust = KpAim * heading_error + min_aim_command;
        }

        double distance_adjust = KpDistance * distance_error;
        SmartDashboard.putNumber("Distance_Adjust", distance_adjust);
        SmartDashboard.putNumber("Steering_Adjust", steering_adjust);


        double left_command  = distance_adjust + steering_adjust;
        double right_command = distance_adjust - steering_adjust;

        double scaled_left_command = (left_command + offset) / range;
        double scaled_right_command = (right_command + offset) / range;

        /*
        final double CLOSE_TO_TARGET_THRESHOLD = 0.2;
        final double CLOSE_TO_TARGET_POWER = 0.3;

        if (scaled_left_command <= CLOSE_TO_TARGET_THRESHOLD && scaled_right_command <= CLOSE_TO_TARGET_THRESHOLD) {
            // Turns out we're close, it's time for Zac's sensors to take over.
            if (m_hatchGrab.IsPanelOnBothSides()) {
                // We made it! Stop. 
            } else if (m_hatchGrab.IsPanelOnLeft()) {
                // we're almost there. Give some juice to the right hand side.
                scaled_right_command = CLOSE_TO_TARGET_POWER;
                scaled_left_command = 0.0;
            } else if (m_hatchGrab.IsPanelOnRight()) {
                scaled_right_command = 0.0;
                scaled_left_command = CLOSE_TO_TARGET_POWER;
            } else {
                scaled_right_command = CLOSE_TO_TARGET_POWER;
                scaled_left_command = CLOSE_TO_TARGET_POWER;
            }
        }
        */
        
        SmartDashboard.putNumber("LeftCommand", scaled_left_command);
        SmartDashboard.putNumber("RightCommand", scaled_right_command);

        lowTransmission();
        m_drive.tankDrive(scaled_left_command, scaled_right_command);
    }

    public void lowTransmission() {
        if (m_transmission != null) {
            m_transmission.set(false);
        }
    }
}

class History {
    History(int historyLenMs, boolean useAbsValue) {
        m_historyLenMs = historyLenMs;
        m_transHistory = new ArrayDeque<TimeEntry>();
        m_useAbsValue = useAbsValue;
    }

    public void appendToHistory(double magnitude) {
        long now = System.currentTimeMillis();
        while (m_transHistory.peekFirst() != null && (now - m_transHistory.peekFirst().time) > m_historyLenMs) {
            m_transHistory.pollFirst();
        }
        m_transHistory.addLast(new TimeEntry(now, m_useAbsValue ? Math.abs(magnitude) : magnitude));
    }

    public double getHistoryAverage() {
        double sum = 0.0;
        if (m_transHistory.size() < 1) {
            return sum;
        }
        for (TimeEntry e : m_transHistory) {
            sum += e.magnitude;
        }

        return sum / m_transHistory.size();
    }

    public double getHistoryAverageWithSalt(double salt) {
        int size = 0;
        size = m_transHistory.size();
        return (getHistoryAverage() * size + salt) / (size + 1);
    }

    public void clear() {
        m_transHistory.clear();
    }

    private class TimeEntry {
        public TimeEntry(long time_, double magnitude_) {
            time = time_;
            magnitude = magnitude_;
        }

        public long time;
        public double magnitude;
    }

    private ArrayDeque<TimeEntry> m_transHistory;
    private int m_historyLenMs;
    private boolean m_useAbsValue;
}

class LED
{
    private Solenoid m_led;

    public LED(int port)
    {
        m_led = new Solenoid(port);
    }

    public void on()
    {
        m_led.set(true);
    }

    public void off()
    {
        m_led.set(false);
    }
}


