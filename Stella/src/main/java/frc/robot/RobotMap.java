package frc.robot;

public class RobotMap {
    
    public static class TalonPort {
        public static final int TEST_MOTOR      = 0;
    }

    public static class digitalInputs{
        public static final int CLAW_SWITCH = 0;
    }
    
    public static class SparkPort {
        public static final int LEFT_DRIVE1     = 1;
        public static final int LEFT_DRIVE2     = 2;
        public static final int RIGHT_DRIVE3    = 3;
        public static final int RIGHT_DRIVE4    = 4;
    }

    public static class VictorPort {
        public static final int IntakeArm   = 5;
        public static final int Roller      = 6;
        public static final int Shooter     = 7; 
    }

    public static class SolenoidPort {
        public static final int DRIVE_TRANS_LOW   =  1;
        public static final int DRIVE_TRANS_HIGH  =  2;
        public static final int INTAKE_DOWN       = -1;
        public static final int INTAKE_UP         = -1;
        public static final int HATCH_GRAB        = -1;
    }

    public static class CompressorPort {
        public static final int MAIN_COMPRESSOR   = 0;
    }

  /*  public static class EncoderPort {
        public static final int One_A = 1;
        public static final int One_B = 2;
        public static final int Two_A = 3;
        public static final int Two_B = 4;
    }
    */
    public static class JoystickPort {
        public static final int Joystick_Port  = 0;
    }
    
   public static enum AutoMode {
        TEST, MOVE_FORWARD, MOVE_BACKWARD
    };
}