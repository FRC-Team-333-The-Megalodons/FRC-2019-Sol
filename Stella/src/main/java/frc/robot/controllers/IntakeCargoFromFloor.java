package frc.robot.controllers;

import frc.robot.subsystems.*;

public class IntakeCargoFromFloor
{
    private final int INTAKE_STATE_BALL_IN_POSSESSION = 0;
    private final int INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN = 1;
    private final int INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT      = 2;
    private final int INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN     = 3;
    private final int INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT    = 4;

    private RobotMech m_mech;
    private RobotArm  m_arm;

    public IntakeCargoFromFloor(RobotMech mech, RobotArm arm)
    {
        m_mech = mech;
        m_arm = arm;
    }

    public int evaluateCurrentState()
    {
        if (m_mech.isCargoPresent()) {
            return INTAKE_STATE_BALL_IN_POSSESSION;
        }
        if (m_arm.isArmAtLow()) {
            if (m_mech.isNoseActuallyOut()) {
                return INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT;
            } else {
                return INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN;
            }
        } else {
            if (m_mech.isNoseActuallyOut()) {
                return INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT;
            } else {
                return INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN;
            }
        }
    }

    // Will return true when we have a ball (at which point it will have turned off the rollers, but done no other movement)
    public boolean do_intake()
    {
        int state = evaluateCurrentState();
        switch (state) {
            case INTAKE_STATE_BALL_IN_POSSESSION: {
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return true;
            }
            // These next two are the same action:
            case INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN:
            case INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN: {
                m_mech.pushNoseOut();
                return false;
            }
            case INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT: {
                m_arm.moveArmDown();
                return false;
            }
            case INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT: {
                m_arm.stopArm();
                m_mech.pullInIntakeRollers();
                m_mech.pullInShooterRollers();
                return false;
            }
        }
        return false;
    }
}