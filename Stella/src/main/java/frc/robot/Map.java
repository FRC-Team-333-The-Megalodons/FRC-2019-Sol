package frc.robot;

public class Map {
    
    public static class TalonPort {
        public static final int TEST_MOTOR      = 0;
    }
    
    public static class SparkPort {
        public static final int LEFT_DRIVE1     = 1;
        public static final int RIGHT_DRIVE3     = 2;
        public static final int LEFT_DRIVE2    = 3;
        public static final int RIGHT_DRIVE4    = 4;
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