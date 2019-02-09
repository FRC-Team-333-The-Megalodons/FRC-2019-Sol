package frc.robot;

public class RobotMap {

    public static class DigitalInputPort {
        public static final int CLAW_SWITCH = 0;
    }

    public static class AnalogPort {
        public static final int ULTRASONIC_SENSOR = 0;
    }
    
    public static class SparkPort {
        public static final int LEFT_DRIVE1     = 1;
        public static final int LEFT_DRIVE2     = 2;
        public static final int RIGHT_DRIVE3    = 3;
        public static final int RIGHT_DRIVE4    = 4;
    }

    public static class TalonPort {
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

    public static class PlayerButton {
        public static final int INTAKE          = 3;
        public static final int SHOOTER         = 4;
        public static final int PUSH_ROLLER_OUT = 6;
        public static final int ARM_UP          = 7;
        public static final int ARM_DOWN        = 9;
        public static final int ROLLER_TOGGLE   = 11;
        public static final int FORCE_LOW_TRANS = 12;
    }

}