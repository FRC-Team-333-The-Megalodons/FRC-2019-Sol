package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class IntakeCargoFromFloor
{
    private final int INTAKE_STATE_HAVE_CARGO_NOSE_IN              = 0;
    private final int INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_DOWN   = 1;
    private final int INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_MIDDLE = 2;
    private final int INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN       = 3;
    private final int INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT      = 4;
    private final int INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN     = 5;
    private final int INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT    = 6;
    private final int INTAKE_STATE_CARGO_RECENTLY_CONSUMED         = 7;

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
            if (m_mech.wasCargoRecentlyConsumed()) {
                return INTAKE_STATE_CARGO_RECENTLY_CONSUMED;
            }
            if (m_mech.isNoseActuallyOut()) {
                if (m_arm.isArmAtTarget(RobotArm.CARGO_TRAVEL_POSITION)) {
                    return INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_MIDDLE;
                } else {
                    return INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_DOWN;
                }
            } else {
                return INTAKE_STATE_HAVE_CARGO_NOSE_IN;
            }
        }
        if (m_arm.isArmAtTarget(RobotArm.BOTTOM_POSITION)) {
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
        SmartDashboard.putNumber("Current Intake-from-floor State", state);
        switch (state) {
            case INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_MIDDLE: {
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return true;
            }
            case INTAKE_STATE_HAVE_CARGO_NOSE_OUT_CLAW_DOWN: {
                m_arm.periodic(RobotArm.CARGO_TRAVEL_POSITION);
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return false;
            }
            
            // These next three collapse into the same action:
            case INTAKE_STATE_HAVE_CARGO_NOSE_IN: 
            case INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_IN:
            case INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_IN:
            {
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                m_mech.pushNoseOut();
                return false;
            }
            case INTAKE_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT: {
                m_arm.periodic(RobotArm.BOTTOM_POSITION);
                return false;
            }
            case INTAKE_STATE_CARGO_RECENTLY_CONSUMED: {
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.pullInShooterRollers();
                return false;
            }
            case INTAKE_STATE_CLAW_IS_DOWN_AND_NOSE_IS_OUT: {
                m_arm.stopArm();
                m_mech.pullInIntakeRollers();
                m_mech.pullInShooterRollers();
                return false;
            }
            default: {
                DriverStation.reportError("UNHANDLED IntakeCargoFromFloorState = "+state+"\n",false);
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return false;
            }
        }
    }
}
