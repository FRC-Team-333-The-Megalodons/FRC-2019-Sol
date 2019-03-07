package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class EjectCargoToFloor
{
    private final int EJECT_STATE_HAVE_NO_CARGO = 0;
    private final int EJECT_STATE_JUST_EJECTED_CARGO = 1;
    private final int EJECT_STATE_CLAW_IS_DOWN = 2;
    private final int EJECT_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT = 3;
    private final int EJECT_STATE_CLAW_IS_UP_AND_NOSE_IS_IN = 4;

    private RobotMech m_mech;
    private RobotArm  m_arm;

    public EjectCargoToFloor(RobotMech mech, RobotArm arm)
    {
        m_mech = mech;
        m_arm = arm;
    }

    public int evaluateCurrentState()
    {
        if (!m_mech.isCargoPresent()) {
            if (m_mech.wasCargoRecentlyShot()) {
                return EJECT_STATE_JUST_EJECTED_CARGO;
            }
            return EJECT_STATE_HAVE_NO_CARGO;
        } 

        if (m_arm.isArmAtTarget(RobotArm.BOTTOM_POSITION)) {
            return EJECT_STATE_CLAW_IS_DOWN;
        }

        if (m_mech.isNoseActuallyOut()) {
            return EJECT_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT;
        }

        return EJECT_STATE_CLAW_IS_UP_AND_NOSE_IS_IN;
    }

    // Will return true when we have a ball (at which point it will have turned off the rollers, but done no other movement)
    public boolean do_eject()
    {
        int state = evaluateCurrentState();
        SmartDashboard.putNumber("Current Eject-to-floor State", state);
        switch (state) {
            case EJECT_STATE_CLAW_IS_UP_AND_NOSE_IS_IN: {
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                m_mech.pushNoseOut();
                return false;
            }
            case EJECT_STATE_CLAW_IS_UP_AND_NOSE_IS_OUT: {
                m_arm.periodic(RobotArm.BOTTOM_POSITION);
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return false;
            }

            case EJECT_STATE_JUST_EJECTED_CARGO:
            case EJECT_STATE_CLAW_IS_DOWN: {
                m_arm.stopArm();
                m_mech.pushOutShooterRollers();
                m_mech.pushOutIntakeRollers();
                return false;
            }

            case EJECT_STATE_HAVE_NO_CARGO: {
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return true;
            }
            default: {
                DriverStation.reportError("UNHANDLED EjectCargoToFloorState = "+state+"\n",false);
                m_arm.stopArm();
                m_mech.stopIntakeRollers();
                m_mech.stopShooterRollers();
                return false;
            }
        }
    }
}
