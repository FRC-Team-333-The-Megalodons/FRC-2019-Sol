package frc.robot.controllers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;

public class ShootCargoInShipFace
{
    private final int STATE_NO_CARGO          = 0;
    private final int STATE_ARM_DOWN_NOSE_IN  = 1;
    private final int STATE_ARM_DOWN_NOSE_OUT = 2;
    private final int STATE_ARM_UP_NOSE_OUT   = 3;
    private final int STATE_ARM_UP_NOSE_IN    = 4;
    private final int STATE_CREEP_LEFT_COMPLETE = 5;
    private final int STATE_CREEP_RIGHT_COMPLETE = 6;
    private final int STATE_CREEP_COMPLETE    = 7;
    private final int STATE_BALL_SHOT         = 8;

    

    public static final double LEFT_CREEP_DISTANCE = -12.5;
    public static final double RIGHT_CREEP_DISTANCE = 12.5;
    public static final double CREEP_SPEED = 0.1;
    
    private int m_lastState = -1;
    private Double m_initialLeftPosition = null;
    private Double m_initialRightPosition = null;


    private CANSparkMax m_leftNeo, m_rightNeo;
    private RobotMech m_mech;
    private RobotArm  m_arm;

    public ShootCargoInShipFace(RobotMech mech, RobotArm arm, CANSparkMax leftNeo, CANSparkMax rightNeo)
    {
        m_mech = mech;
        m_arm = arm;
        m_leftNeo = leftNeo;
        m_rightNeo = rightNeo;
    }

    private void resetCreepPositions()
    {
        m_initialLeftPosition = null;
        m_initialRightPosition = null;
    }

    public double getLeftPos()
    {
        return m_leftNeo.getEncoder().getPosition();
    }
    public double getRightPos()
    {
        return m_rightNeo.getEncoder().getPosition();
    }

    private void setCreepPositionsIfNull()
    {
        if (m_initialLeftPosition == null) {
            m_initialLeftPosition = getLeftPos();
            m_initialRightPosition = getRightPos();
            System.out.println("ShootCargoInShipFace: Set initial left & right to "+m_initialLeftPosition+" & "+m_initialRightPosition);
        }
    }

    public int evaluateCurrentState_impl(double position, double adjust)
    {
        if (m_mech.wasCargoRecentlyShot()) {
            resetCreepPositions();
            return STATE_BALL_SHOT;
        }

        if (!m_mech.isCargoPresent()) {
            resetCreepPositions();
            return STATE_NO_CARGO;
        }

        if (m_arm.isArmAtTarget(position)) {
            if (m_mech.isNoseActuallyOut()) {
                resetCreepPositions();
                return STATE_ARM_UP_NOSE_OUT;
            } else {
                // CREEP section. Only section we don't reset the Creep Initial Readings.
                setCreepPositionsIfNull();
                
                boolean leftDone = (getLeftPos() - m_initialLeftPosition) < LEFT_CREEP_DISTANCE+adjust;
                boolean rightDone = (getRightPos() - m_initialRightPosition) > RIGHT_CREEP_DISTANCE+adjust;

                if (leftDone && rightDone) {
                    return STATE_CREEP_COMPLETE;
                } else if (leftDone) {
                    return STATE_CREEP_LEFT_COMPLETE;
                } else if (rightDone) {
                    return STATE_CREEP_RIGHT_COMPLETE;
                }
                return STATE_ARM_UP_NOSE_IN;
            }
        } else {
            if (m_mech.isNoseActuallyOut()) {
                resetCreepPositions();
                return STATE_ARM_DOWN_NOSE_OUT;
            } else {
                resetCreepPositions();
                return STATE_ARM_DOWN_NOSE_IN;
            }
        }

    }

    public int evaluateCurrentState(double position, double adjust)
    {
        int state = evaluateCurrentState_impl(position, adjust);
        if (m_lastState != state) {
            System.out.println("ShootCargoInShipFace: previous="+m_lastState+", new="+state);
        }
        m_lastState = state;
        return m_lastState;
    }

    // Will return true when we have a ball (at which point it will have turned off the rollers, but done no other movement)
    public boolean do_drool(double position, double adjust)
    {
        int state = evaluateCurrentState(position, adjust);
        switch (state) {
            case STATE_NO_CARGO: {
                m_arm.stopArm();
                m_mech.stopShooterRollers();
                return false;
            }
            case STATE_BALL_SHOT:
            case STATE_CREEP_COMPLETE: {
                m_arm.stopArm();
                m_leftNeo.set(0.0);
                m_rightNeo.set(0.0);
                m_mech.droolOutShooterRollers();
                return false;
            }
            case STATE_CREEP_LEFT_COMPLETE: {
                m_arm.stopArm();
                m_leftNeo.set(0.0);
                m_rightNeo.set(CREEP_SPEED);
                return false;
            }
            case STATE_CREEP_RIGHT_COMPLETE: {
                m_arm.stopArm();
                m_rightNeo.set(0.0);
                m_leftNeo.set(-CREEP_SPEED);
                return false;
            }
            case STATE_ARM_UP_NOSE_IN: {
                m_arm.stopArm();
                m_rightNeo.set(CREEP_SPEED);
                m_leftNeo.set(-CREEP_SPEED);
                return false;
            }
            case STATE_ARM_UP_NOSE_OUT: {
                m_arm.stopArm();
                m_mech.pullNoseIn();
                return false;
            }
            case STATE_ARM_DOWN_NOSE_OUT: {
                m_arm.periodic(position);
                return false;
            }
            case STATE_ARM_DOWN_NOSE_IN: {
                m_arm.stopArm();
                m_mech.pushNoseOut();
                return false;
            }
            default: {
                DriverStation.reportError("UNHANDLED ShootCargoInShipFace = "+state+"\n",false);
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return false;
            }
        }
    }
}
