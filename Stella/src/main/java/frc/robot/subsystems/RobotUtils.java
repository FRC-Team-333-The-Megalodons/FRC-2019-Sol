package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.ArrayList;

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
import frc.robot.subsystems.RobotMap.PlayerButton;

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

class DriveTrainEncoder implements PIDSource {
    private Encoder m_leftEncoder, m_rightEncoder;

    public DriveTrainEncoder(Encoder left, Encoder right) {
        m_leftEncoder = left;
        m_rightEncoder = right;
    }

    public double getLeftDistance() {
        return m_leftEncoder.getDistance();
    }

    public double getRightDistance() {
        return m_rightEncoder.getDistance();
    }

    public double averageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
    }

    public double getLeftRaw() {
        return m_leftEncoder.getRaw();
    }

    public double getRightRaw() {
        return m_rightEncoder.getRaw();
    }

    public double getLeftRate() {
        return m_leftEncoder.getRate();
    }

    public double getRightRate() {
        return m_rightEncoder.getRate();
    }

    public double leftPidGet() {
        return m_leftEncoder.pidGet();
    }

    public double rightPidGet() {
        return m_rightEncoder.pidGet();
    }

    public void reset() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public void setDistancePerPulse(double value) {
        m_leftEncoder.setDistancePerPulse(value);
        m_rightEncoder.setDistancePerPulse(value);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        m_leftEncoder.setPIDSourceType(pidSource);
        m_rightEncoder.setPIDSourceType(pidSource);
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        if (m_leftEncoder.getPIDSourceType() != m_rightEncoder.getPIDSourceType()) {
            DriverStation.reportError("PID SOURCE TYPE MISMATCH!!!", false);
        }
        return m_leftEncoder.getPIDSourceType();
    }

    @Override
    public double pidGet() {
        return (m_leftEncoder.pidGet() + m_rightEncoder.pidGet()) / 2.0;
    }
}

class MotorEncoder {
    private Encoder m_encoder;
    private boolean m_inverse;

    public MotorEncoder(Encoder encoder) {
        m_encoder = encoder;
        // m_inverse = inverse;
    }

    public double getDistance() {
        return m_encoder.getDistance() * (m_inverse ? -1 : 1.0);
    }

    public void reset() {
        m_encoder.reset();
    }

    public double getRaw() {
        return m_encoder.get();
    }

    public void setDistancePerPulse(double distancePerPulse) {
        m_encoder.setDistancePerPulse(distancePerPulse);
    }

    public double getRate() {
        return m_encoder.getRate();
    }
}

// This class is a significantly simplified version of a PID controller,
// that re-evalutes only at the time of "setSetpoint" rather than in a
// background thread.
class SimplePIDController {

    // Assume that min & max are absolute values, and can be negative on our real
    // output device.
    public SimplePIDController(PIDSource source, PIDOutput output, DigitalInput lowerLimitSwitch,
            DigitalInput upperLimitSwitch, // Limit Switches (can be null)
            double abs_output_min, double abs_output_max, // Output Min/Max
            double threshold_delta, double tolerance, int history_len_ms) {

        m_transHistory = new History(history_len_ms, false);
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

        m_transHistory.appendToHistory(m_source.pidGet());
        double current = m_transHistory.getHistoryAverage();
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

        // System.out.println("PIDWRITE "+output+" (sign=" + sign +", cap="+
        // m_abs_output_max+")");
        m_output.pidWrite(output);
    }

    private double m_abs_output_min, m_abs_output_max, m_threshold_delta, m_tolerance, m_speedModifier;
    private Double m_target;
    private PIDSource m_source;
    private PIDOutput m_output;
    private DigitalInput m_lowerLimitSwitch, m_upperLimitSwitch;
    private History m_transHistory;
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
    private History m_transHistory, m_speedHistory;
    private DifferentialDrive m_drive;
    private Solenoidal m_transmission;

    public TeleopTransDrive(DifferentialDrive drive, Solenoidal transmission, int button_forceLowTrans) {
        m_transmission = transmission;
        m_drive = drive;
        m_drive.setExpiration(0.1);
        m_drive.setSafetyEnabled(true);
        m_drive.setMaxOutput(1.0);
        m_button_forceLowTrans = button_forceLowTrans;
        m_transHistory = new History(TRANS_HISTORY_LEN_MS, true);
        m_speedHistory = new History(SPEED_HISTORY_LEN_MS, false);
    }

    public void arcadeDrive(Joystick stick, Double abs_limit) {

        double joystick_Y = stick.getY();
        double joystick_X = stick.getX();

        // If the elevator is up, just limit our inputs.
        if (abs_limit != null) {
            joystick_Y = RobotUtils.abs_min(joystick_Y, abs_limit.doubleValue());
            joystick_X = RobotUtils.abs_min(joystick_X, abs_limit.doubleValue());
        }

        // Even if Auto isn't on, we should be keeping track of the history so we can
        // turn it on later.
        m_transHistory.appendToHistory(joystick_Y);
        m_speedHistory.appendToHistory(joystick_Y);

        // See whether the differential in Speed will be too much and the robot will flip.
        /*
        final double speed_delta_limit = 0.5;
        double speedHistoryAverage = m_speedHistory.getHistoryAverage();
        if (Math.abs(joystick_Y - speedHistoryAverage) > speed_delta_limit) {
            // The change is too much! Limit it down to only speed_delta_limit away.
            double sign = Math.signum(joystick_Y);
            double new_Y = speedHistoryAverage + (speed_delta_limit * sign);
            System.out.println("History="+speedHistoryAverage+"; Y="+joystick_Y+"; Sign="+sign+"; new_Y="+new_Y);
            joystick_Y = new_Y;
        }
        */

        // Actually put the input into the drivetrain
        m_drive.arcadeDrive(joystick_Y, -joystick_X);

        // If the driver is holding the Low Transmission button, force it into low transmission.
        if (stick.getRawButton(m_button_forceLowTrans)) {
            if (lowTransmission()) {
                System.out.println("Force-Switching to Low Transmission");
            }
            return;
        }

        double transHistoryAverage = m_transHistory.getHistoryAverage();

        //TODO: Making changees to Auto Shifting here. The robot should always be in HIGH unless button 2 is pressed.
        //Mike changed the Gear Ratio

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

    /*ADDED BY ZAC FOR CURVATURE*/
    public static final double CURVATURE_TURN_SCALE = .25;   //scales turns in curvature drive
    public static final double CURVATURE_DOWNSCALE_THRESHOLD = 0.9;

    public void curvatureDrive(Joystick stick, Double abs_limit) {

        // When the low transmission button is held, fall back to arcadeDrive
        if (stick.getRawButton(PlayerButton.FORCE_LOW_TRANSMISSION)) {
            arcadeDrive(stick, abs_limit);
            return;
        }

        double joystick_Y = stick.getY();
        double joystick_X = stick.getX();

        final double CURVATURE_TURN_OVERRIDE_THRESHOLD = 0.9; //if the joystick x (sideways) value is above this pivot turns are possible as long as the driver isn't going forward
        final double CURVATURE_FORWARD_OVERRIDE_THRESHOLD = 0.2;//controlls how much the driver needs to go forward before curvature kicks in
    
        boolean isQuickTurn = Math.abs(stick.getX()) > CURVATURE_TURN_OVERRIDE_THRESHOLD // Should we do an in-place turn?
                                && Math.abs(stick.getY()) < CURVATURE_FORWARD_OVERRIDE_THRESHOLD;

        // If the elevator is up, just limit our inputs.
        if (abs_limit != null) {
            joystick_Y = RobotUtils.abs_min(joystick_Y, abs_limit.doubleValue());
            joystick_X = RobotUtils.abs_min(joystick_X, abs_limit.doubleValue());
        }

        // Even if Auto isn't on, we should be keeping track of the history so we can
        // turn it on later.
        m_transHistory.appendToHistory(joystick_Y);
        m_speedHistory.appendToHistory(joystick_Y);

        double power = Math.pow(joystick_Y, 3);
        //if (isQuickTurn) { power = Math.abs(joystick_X); }

        double rotation = -joystick_X;

        /*
        if (Math.abs(rotation) < CURVATURE_DOWNSCALE_THRESHOLD) {
            rotation *= Math.pow(CURVATURE_TURN_SCALE, 2);
        }
        */

        m_drive.curvatureDrive(power, rotation, isQuickTurn);

        /*

        // Actually put the input into the drivetrain 
        m_drive.curvatureDrive(Math.pow(joystick_Y, 3),//scale the speed of the joystick going forward by raising it to the 3rd power
                               (joystick_X >= .9 ? 
                                    -joystick_X : 
                                    (stick.getRawButton(PlayerButton.FORCE_NO_CURVATURE) ? 
                                        -CURVATURE_TURN_SCALE*joystick_X : 
                                        -CURVATURE_TURN_SCALE*.5*joystick_X)), //scale the joystick under a threashold
                               (curvatureDriveEnabled || 
                                    (Math.abs(joystick_X) > CURVATURE_TURN_OVERRIDE_THRESHOLD && Math.abs(joystick_Y) < CURVATURE_FORWARD_OVERRIDE_THRESHOLD))); // use curvature drive as long as the buttons arent being pressed and the driver isn't just going to the side
        */

        // If the driver is holding the Low Transmission button, force it into low transmission.
        if (stick.getRawButton(m_button_forceLowTrans)) {
            if (lowTransmission()) {
                System.out.println("Force-Switching to Low Transmission");
            }
            return;
        }

        double transHistoryAverage = m_transHistory.getHistoryAverage();

        // If the driver is holding the joystick below the Low Transmission
        //   threshold for long enough, downshift into Low Gear.
        /*if (transHistoryAverage < LOW_THRESHOLD) {
            if (lowTransmission()) {
                System.out.println("Auto-Switching to Low Transmission");
            }
            return;
        }*/
        // If the driver is holding the joystick above the High Transmission 
        //   threshold for long enough, upshift into High Gear.
        if (transHistoryAverage >= HIGH_THRESHOLD) {
            if (highTransmission()) {
                System.out.println("Auto-Switching to High Transmission");
            }
            return;
        }
        
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
    
    private DifferentialDrive m_drive;
    private Solenoidal m_transmission;
    private RobotHatchGrab m_hatchGrab;
    private RobotCargoState m_cargoState;

    public LimelightDrive(DifferentialDrive drive, Solenoidal transmission, RobotHatchGrab hatchGrab, RobotCargoState cargoState) {
        m_transmission = transmission;
        m_drive = drive;
        m_hatchGrab = hatchGrab;
        m_cargoState = cargoState;

        //m_drive.setExpiration(0.1);
        //m_drive.setSafetyEnabled(true);
        //m_drive.setMaxOutput(0.5);
    }

    public void autoDrive(int limelightPipeline, double tx, double ty, double area, boolean rocketMode) {

        // Initialize configurations to RobotMap.LimelightPipeline.HATCH values.
        double KpAim = kAIM; //SmartDashboard.getNumber("AutoDrive_kAIM", kAIM);
        double KpDistance = kDistance; //SmartDashboard.getNumber("AutoDrive_kDistance", kDistance);
        double min_aim_command = kMinInc; //SmartDashboard.getNumber("AutoDrive_minInc", 0.05f);
        double max_area = 6.0;
        if (m_cargoState.isCargoPresent()) {
            max_area = 5.0;
            if (rocketMode) {
              max_area = 4.0;
            }
        }
        double min_area = 0.0;

        // If we're actually in CARGO hunting mode, then tweak our values.
        if (limelightPipeline == RobotMap.LimelightPipeline.CARGO) {

        }

        
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
