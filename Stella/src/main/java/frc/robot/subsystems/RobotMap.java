package frc.robot.subsystems;

public class RobotMap {

    
    public static class PlayerButton {
        public static final int FORCE_LOW_TRANSMISSION = 2;
        public static final int INTAKE_CARGO           = 3;
        public static final int FIRE_CARGO             = 4;
        public static final int SPIT_OUT_CARGO         = 6;
        public static final int MOVE_ARM_UP            = 7;
        public static final int CHASE_REFLECTIVE_TAPE  = 8;
        public static final int MOVE_ARM_DOWN          = 9;
        public static final int NOSE_OUT               = 11;
        public static final int NOSE_IN                = 12;
        public static final int FORCE_NO_CURVATURE     = 10;
    }

    public static class DigitalInputPort {
        public static final int CLAW_SWITCH        =  0;
        public static final int UPPER_ARM_SWITCH   =  1;
        public static final int LOWER_ARM_SWITCH   =  2;
    }

    public static class AnalogPort {
        public static final int ULTRASONIC_SENSOR = 0;
        public static final int ARM_POTENTIOMETER = 3;
    }
    
    public static class SparkPort {
        public static final int LEFT_DRIVE1     = 1;
        public static final int LEFT_DRIVE2     = 2;
        public static final int RIGHT_DRIVE3    = 3;
        public static final int RIGHT_DRIVE4    = 4;
    }
    
    public static class CANSparkID {
        public static final int LEFT_LEADER     = 0;
        public static final int LEFT_FOLLOWER   = 1;
        public static final int RIGHT_LEADER    = 2;
        public static final int RIGHT_FOLLOWER  = 3;
    }

    public static class VictorPort {
        public static final int IntakeArm   = 5;
        public static final int Roller      = 6;
        public static final int Shooter     = 7; 
    }

    public static class SolenoidPort {
        public static final int DRIVE_TRANS_1     = 1;
        public static final int DRIVE_TRANS_2     = 2;
        public static final int ROLLER_UP         = 3;
        public static final int ROLLER_DOWN       = 4;
        public static final int HATCH_GRAB        = 5;
    }

    public static class CompressorPort {
        public static final int MAIN_COMPRESSOR   = 0;
    }

    public static class JoystickPort {
        public static final int Joystick_Port  = 0;
    }
    
   public static enum AutoMode {
        TEST, MOVE_FORWARD, MOVE_BACKWARD
    };


}