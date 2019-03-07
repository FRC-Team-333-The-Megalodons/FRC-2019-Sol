package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;

public class ShootCargoIntoShip
{
    private final int STATE_NO_CARGO              = 0;
    private final int STATE_NOSE_IN_CLAW_DOWN     = 1;
    private final int STATE_NOSE_OUT_CLAW_DOWN    = 2;
    private final int STATE_CLAW_UP               = 3;
    private final int STATE_BALL_SHOT             = 4;
    
    private int m_lastState = -1;
    private RobotMech m_mech;
    private RobotArm  m_arm;

    public ShootCargoIntoShip(RobotMech mech, RobotArm arm)
    {
        m_mech = mech;
        m_arm = arm;
    }

    public int evaluateCurrentState_impl(position)
    {
        if (m_mech.wasCargoRecentlyShot()) {
            return STATE_BALL_SHOT;
        }

        if (!m_mech.isCargoPresent()) {
            return STATE_NO_CARGO;
        }

        if (m_mech.isNoseActuallyOut()) {
            if (m_arm.isArmAtTarget(position)) {
                return STATE_CLAW_UP;
            } else {
                return STATE_NOSE_OUT_CLAW_DOWN;
            }
        } else {
            return STATE_NOSE_IN_CLAW_DOWN;
        }
    }

    public int evaluateCurrentState(double position)
    {
        int state = evaluateCurrentState_impl(position);
        if (m_lastState != state) {
            System.out.println("ShootCargoIntoShip: previous="+m_lastState+", new="+state);
        }
        m_lastState = state;
        return m_lastState;
    }

    // Will return true when we have a ball (at which point it will have turned off the rollers, but done no other movement)
    public boolean do_shoot(double position)
    {
        int state = evaluateCurrentState(position);
        switch (state) {
            case STATE_NO_CARGO: {
                m_arm.stopArm();
                m_mech.stopShooterRollers();
                return false;
            }
            case STATE_NOSE_IN_CLAW_DOWN: {
                // need to push the claw out
                m_mech.pushNoseOut();
                return false;
            }
            case STATE_NOSE_OUT_CLAW_DOWN: {
                m_arm.periodic(position);
                return false;
            }
            case STATE_CLAW_UP: 
            case STATE_BALL_SHOT: {
                m_arm.stopArm();
                m_mech.pushOutShooterRollers();
                return false;
            }
            default: {
                DriverStation.reportError("UNHANDLED IntakeCargoFromHumanState = "+state+"\n",false);
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return false;
            }
        }
    }
}
