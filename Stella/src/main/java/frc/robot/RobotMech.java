package frc.robot;

//import frc.robot.Map.TalonPort;
import frc.robot.RobotMap.SparkPort;
import edu.wpi.first.wpilibj.DigitalInput;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import frc.robot.RobotMap.VictorPort;
import frc.robot.RobotMap.digitalInputs;

public class RobotMech {
    
    public static final double MOTOR_POWER = 0.4;

    public RobotMech() {
        
        try {
            m_intakeArm = new Arm(new Victor(VictorPort.IntakeArm));
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Arm\n", false);
        }

        try {
            m_roller = new Roller(new Victor(VictorPort.Roller));
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Roller\n", false);
        }

        try {
            m_shooter = new Shooter(new Victor(VictorPort.Shooter));
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate the Shooter\n", false);
        }

        m_clawSwitch = new DigitalInput(digitalInputs.CLAW_SWITCH);
    

     /*   //Instantiate Test Motor w/ Encoder
        try {
            talon = new TalonSRX(TalonPort.TEST_MOTOR);
        } catch(Exception ex) {
            DriverStation.reportError("Could not instatiate test motor\n", false);
        }
     */   
  /*      //Instantiate Test Motor
        try {
            motor = new Spark(SparkPort.MOTOR_A);
        }catch (Exception ex) {
            DriverStation.reportError("Coudn't instantitate Motor_A",false);
        }
     */   
    }

    
    public void periodic(Joystick stick) {
        if(stick == null) {
            DriverStation.reportError("No Joystick, cannot run Mech periodic\n", false);
            return;
        }

        //ARM : PWM 6
        // 7: Raise arm up
        // 9: Lower arm down
        if (stick.getRawButton(7)) {
            m_intakeArm.m_victor.set(.75);
        } else if (stick.getRawButton(9)) {
            m_intakeArm.m_victor.set(-.10);
        } else {
            m_intakeArm.stop();
        }

        //ROLLER : PWM 6
        // 5: Take cargo in
        // 6: Give ball out
        if (stick.getRawButton(5)) {
            m_roller.m_victor.set(1);
        } else if (stick.getRawButton(6)) {
            m_roller.m_victor.set(-1);
        } else {
            m_roller.stop();
        }

        //SHOOTER : PWM 7
        // 3: Take cargo in
        // 4: Shoot ball out
        if (stick.getRawButton(3)) {
            //m_shooter.m_victor.set(-.5);
            System.out.print(m_clawSwitch.get());
            if (!m_clawSwitch.get()) {
                m_shooter.m_victor.set(1);
            } else {
                m_shooter.m_victor.set(-1);
            }
        }else{
            m_shooter.stop();
        }



        
       /* if (stick.getRawButton(2)) {
            motor.set(MOTOR_POWER);
        } else if (stick.getTrigger()) {
            talon.set(ControlMode.PercentOutput, MOTOR_POWER); //how to give power to motors using the Talon SRX
        } else {
            talon.set(ControlMode.PercentOutput, 0.0);
            motor.set(0);
        }
      */  
        //testMotor.getMotionProfileStatus(); 
    }
    class MechController {

        public MechController(Arm intake) {
            m_intakeArm = intake;
        }

        public void stop() {
            m_intakeArm.stop();
        }
    }
    
    class Arm {
        public Arm(Victor victor) {
            m_victor = victor;
        }



        public void stop() {
            m_victor.set(0);
        }
        Victor m_victor;
    }

    class Roller {
        public Roller(Victor victor) {
            m_victor = victor;
        }

        public void stop() {
            m_victor.set(0);
        }
        Victor m_victor;
    }

    class Shooter {
        public Shooter(Victor victor) {
            m_victor = victor;
        }

        public void stop() {
            m_victor.set(0);
        }
        Victor m_victor;
    }
 
    //private TalonSRX talon;
    private Arm  m_intakeArm;
    private Roller m_roller;
    private Shooter m_shooter;
    private DigitalInput m_clawSwitch;
        
    }
    

