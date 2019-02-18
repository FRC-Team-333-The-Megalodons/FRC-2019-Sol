/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */                                                                                          
/* must be accompanied by the FIRST BSD license file in the root directory of */                                                                                                
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;                            

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RobotMap.AnalogPort;

public class RobotHatchPanelSensor {

    private HatchPanelSensor m_rightPanelSensor;
    private HatchPanelSensor m_leftPanelSensor;

    public static final double FULLYON = -1;
    public static String whichSide;


    public RobotHatchPanelSensor() {
        try {
            m_rightPanelSensor = new HatchPanelSensor(new AnalogInput(AnalogPort.RIGHT_PANEL_SENSOR));   
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate Left panel sensor\n", false);
        }

        try {
            m_leftPanelSensor = new HatchPanelSensor(new AnalogInput(AnalogPort.LEFT_PANEL_SENSOR));
        } catch (Exception e) {
            DriverStation.reportError("Could not instantiate Left panel sensor\n", false);
        }
    
    }

    public boolean IsPanelOnLeft() {
        return m_leftPanelSensor.getState();
    }


    public boolean IsPanelOnRight() {
        return m_rightPanelSensor.getState();
    }

    public boolean IsPanelOnBothSides() {
        return ((m_leftPanelSensor.getState() == true) && (m_rightPanelSensor.getState() == true));
    }

    public void UpdatewhichSideIsOff() {
        if ((m_leftPanelSensor.getState() == false) && (m_rightPanelSensor.getState() == false)) {
            whichSide = "BOTH";
        } else if (m_leftPanelSensor.getState() == false) {
            whichSide = "LEFT";
        } else if (m_rightPanelSensor.getState() == false) {
            whichSide = "RIGHT";
        } else {
            whichSide = "-";
        }

        SmartDashboard.putString("Which Side Is Off:", whichSide);
    }

class HatchPanelSensor{

    AnalogInput PanelSensorPort;
    public static final int HAS_HATCH_PANEL_VALUE  = 100;

     public HatchPanelSensor(AnalogInput PanelSensorPort){
             this.PanelSensorPort = PanelSensorPort;
     }

     public boolean getState(){
         return PanelSensorPort.getValue() >= HAS_HATCH_PANEL_VALUE;
     }

}
}
