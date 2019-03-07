package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class IntakeCargoFromHuman
{
    private final int INTAKE_STATE_HAVE_CARGO_NOSE_IN              = 0;
    private final int INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_UP     = 1;
    private final int INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_MIDDLE = 2;
    private final int INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN       = 3;
    private final int INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT      = 4;
    private final int INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN     = 5;
    private final int INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT    = 6;
    private final int INTAKE_STATE_CARGO_RECENTLY_CONSUMED         = 7;
    
    private RobotMech m_mech;
    private RobotArm  m_arm;

    public IntakeCargoFromHuman(RobotMech mech, RobotArm arm)
    {
        m_mech = mech;
        m_arm = arm;
    }

    public int evaluateCurrentState()
    {
        if (m_mech.isCargoPresent()) {
            if (m_mech.wasCargoRecentlyConsumed()) {
              return INTAKE_STATE_CARGO_RECENTLY_CONSUMED;
            }
            if (m_mech.isNoseActuallyOut()) {
                if (m_arm.isArmAtTarget(RobotArm.CARGO_TRAVEL_POSITION)) {
                    return INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_MIDDLE;
                } else {
                    return INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_UP;
                }
            } else {
                return INTAKE_STATE_HAVE_CARGO_NOSE_IN;
            }
        } else {
            if (m_arm.isArmAtTarget(RobotArm.TOP_POSITION)) {
                if (m_mech.isNoseActuallyOut()) {
                    return INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT;
                } else {
                    return INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN;
                }
            } else {
                if (m_mech.isNoseActuallyOut()) {
                    return INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT;
                } else {
                    return INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN;
                }
            }
        }
    }

    // Will return true when we have a ball (at which point it will have turned off the rollers, but done no other movement)
    public boolean do_intake()
    {
        int state = evaluateCurrentState();
        SmartDashboard.putNumber("Current Intake-from-human State", state);

        switch (state) {
            case INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_MIDDLE: {
                // We're finally done!
                m_arm.stopArm();
                return true;
            }
            case INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_UP: {
                // Everything else is good, but we still need to move to the 
                //  arm to middle so that the limelight can see over it.
                m_arm.periodic(RobotArm.CARGO_TRAVEL_POSITION);
                return false;
            }
            case INTAKE_STATE_HAVE_CARGO_NOSE_IN: {
                // We just got the ball! Prep to move the arm out.
                m_arm.stopArm();
                m_mech.stopShooterRollers();
                m_mech.getHatchGrab().open();
                m_mech.pushNoseOut();
                return false;
            }
            case INTAKE_STATE_CARGO_RECENTLY_CONSUMED:
            case INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN: {
                m_mech.pullInShooterRollers();
                m_arm.stopArm();
                return false;
            }
            case INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT: {
                m_arm.stopArm();
                m_mech.getHatchGrab().close();
                m_mech.pullNoseIn();
                return false;
            }
            case INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT: {
                m_arm.periodic(RobotArm.TOP_POSITION);
                return false;
            }
            case INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN: {
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
