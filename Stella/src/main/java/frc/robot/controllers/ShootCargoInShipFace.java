package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;

public class ShootCargoInShipFace
{
    private final int STATE_NO_CARGO          = 0;
    private final int STATE_ARM_DOWN_NOSE_IN  = 1;
    private final int STATE_ARM_DOWN_NOSE_OUT = 2;
    private final int STATE_ARM_UP_NOSE_OUT   = 3;
    private final int STATE_ARM_UP_NOSE_IN    = 4;
    private final int STATE_CREEP_COMPLETE    = 5;
    private final int STATE_BALL_SHOT         = 6;
    
    private int m_lastState = -1;
    private RobotMech m_mech;
    private RobotArm  m_arm;

    public ShootCargoInShipFace(RobotMech mech, RobotArm arm)
    {
        m_mech = mech;
        m_arm = arm;
    }

    public int evaluateCurrentState_impl(double position)
    {
        if (m_mech.wasCargoRecentlyShot()) {
            return STATE_BALL_SHOT;
        }

        if (!m_mech.isCargoPresent()) {
            return STATE_NO_CARGO;
        }

        if (m_arm.isArmAtTarget(position)) {
            if (m_mech.isNoseActuallyOut()) {
                return STATE_ARM_UP_NOSE_OUT;
            } else {
                // check in here for whether we have CREEPed yet
                return STATE_ARM_UP_NOSE_IN;
            }
        } else {
            if (m_mech.isNoseActuallyOut()) {
                return STATE_ARM_DOWN_NOSE_OUT;
            } else {
                return STATE_ARM_DOWN_NOSE_IN;
            }
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
    public boolean do_drool(double position)
    {

        //
        m_mech.droolOutShooterRollers();

        if (0 < System.currentTimeMillis()) {
            return false;
        }

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
            case STATE_NOSE_OUT_CLAW_UP: {
                m_mech.getHatchGrab().close();
                m_mech.pullNoseIn();
                m_arm.stopArm();
                return false;
            } 
            case STATE_NOSE_IN_CLAW_UP: 
            case STATE_BALL_SHOT: {
                m_arm.stopArm();
                m_mech.droolOutShooterRollers();
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
