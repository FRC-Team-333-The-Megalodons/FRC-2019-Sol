package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.RobotMap.AnalogPort;

public class RobotHatchGrab
{
    private SolenoidT m_hatchGrab;
    
    private HatchPanelSensor m_rightPanelSensor;
    private HatchPanelSensor m_leftPanelSensor;
    private LED m_indicatorLight;

    public static final double FULLYON = -1;
    public static String whichSide;


    public RobotHatchGrab(int hatchSolenoidPort, int leftPanelSensor, int rightPanelSensor, int indicatorLightPort)
    {

        try {
            m_hatchGrab = new SolenoidT(hatchSolenoidPort);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate hatch panel solenoid\n", false);
        }

        try {
            m_rightPanelSensor = new HatchPanelSensor(new AnalogInput(rightPanelSensor), 3500);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate right panel sensor\n"+ex.toString(), false);
        }

        try {
            m_leftPanelSensor = new HatchPanelSensor(new AnalogInput(leftPanelSensor), 3700);
        } catch (Exception ex) {
            DriverStation.reportError("Could not instantiate left panel sensor\n", false);
        }

        try {
            m_indicatorLight = new LED(indicatorLightPort);
        } catch (Exception ex) {
            DriverStation.reportError("Could not indicator light\n", false);
        }
        
    }

    public void activateIndicatorLight()
    {
        m_indicatorLight.on();
    }

    public void deactivateIndicatorLight()
    {
        m_indicatorLight.off();
    }

    public boolean IsPanelOnLeft() {
        if(m_leftPanelSensor == null){
            return false;
        }
        return m_leftPanelSensor.getState();
    }


    public boolean IsPanelOnRight() {
        return m_rightPanelSensor.getState();
    }

    public boolean IsPanelOnBothSides() {
        return (IsPanelOnLeft() && IsPanelOnRight());
    }

    public double RawValue(){
        return m_leftPanelSensor.getVoltage();
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

    public void open()
    {
        m_hatchGrab.set(false);
    }

    public void close()
    {
        deactivateIndicatorLight();
        m_hatchGrab.set(true);
    }

}

class HatchPanelSensor{

    private AnalogInput m_panelSensorPort;
    public int HAS_HATCH_PANEL_VALUE;

    public HatchPanelSensor(AnalogInput PanelSensorPort, int HAS_HATCH_PANEL_VALUE){
        m_panelSensorPort = PanelSensorPort;
        this.HAS_HATCH_PANEL_VALUE = HAS_HATCH_PANEL_VALUE;
    }

    public boolean getState(){
        return m_panelSensorPort.getValue() > HAS_HATCH_PANEL_VALUE;
    }

    public double getVoltage(){
        return m_panelSensorPort.getValue();
    }

}
