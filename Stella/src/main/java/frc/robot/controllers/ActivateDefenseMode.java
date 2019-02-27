package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.*;

public class ActivateDefenseMode
{
    private final int STATE_CLAW_DOWN_NOSE_IN     = 0;
    private final int STATE_CLAW_DOWN_NOSE_OUT    = 1;
    private final int STATE_CLAW_UP_NOSE_IN       = 2;
    private final int STATE_CLAW_UP_NOSE_OUT      = 3;
    
    private RobotMech m_mech;
    private RobotArm  m_arm;

    public ActivateDefenseMode(RobotMech mech, RobotArm arm)
    {
        m_mech = mech;
        m_arm = arm;
    }

    public int evaluateCurrentState()
    {
        if (!m_mech.isNoseActuallyOut()) {
            if (m_arm.isArmAtTarget(RobotArm.BOTTOM_POSITION)) {
                return STATE_CLAW_DOWN_NOSE_IN;
            } else {
                return STATE_CLAW_UP_NOSE_IN;
            }
        } else {
            if (m_arm.isArmAtTarget(RobotArm.BOTTOM_POSITION)) {
                return STATE_CLAW_DOWN_NOSE_OUT;
            } else {
                return STATE_CLAW_UP_NOSE_OUT;
            }
        }
    }

    // Will return true when we have a ball (at which point it will have turned off the rollers, but done no other movement)
    public boolean do_defense()
    {
        int state = evaluateCurrentState();
        switch (state) {
            case STATE_CLAW_DOWN_NOSE_IN: {
                m_arm.stopArm();
                return true;
            }
            case STATE_CLAW_DOWN_NOSE_OUT: {
                m_arm.stopArm();
                m_mech.pullNoseIn();
                return false;
            }
            case STATE_CLAW_UP_NOSE_OUT: {
                m_arm.periodic(RobotArm.BOTTOM_POSITION);
                return false;
            }
            case STATE_CLAW_UP_NOSE_IN: {
                m_arm.stopArm();
                m_mech.pushNoseOut();
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