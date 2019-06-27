package frc.robot.kalmanfilter;

import com.revrobotics.*;

import edu.wpi.first.wpilibj.AnalogInput;

public class distance_prosses_model extends prosses_model{

    private final double MAX_CONFIDENCE  = 1;
    private final double VELOCITY_TO_CONFIDENCE  = .125;//TODO make this a real value
    private final double WheelDiam = 5;

    private CANSparkMax leftDrive, rightDrive;
    private AnalogInput ultrasonicSensor;
    private double voltageToInches;
    private double previousPositionLeft, previousPositionRight;
    

    public distance_prosses_model(CANSparkMax leftDrive, CANSparkMax rightDrive, AnalogInput ultrasonicSensor, double voltageToInches){
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.ultrasonicSensor = ultrasonicSensor;
        this.voltageToInches = voltageToInches;
    }

    @Override
    public Gaussian update(double t) {
        double expectedStep;
        double confidence;
        if(leftDrive.getEncoder().getVelocity() == rightDrive.getEncoder().getVelocity()){
            expectedStep = (leftDrive.getEncoder().getPosition()-previousPositionLeft)*Math.PI*WheelDiam;
            confidence = MAX_CONFIDENCE;
        }else{
            expectedStep = ((leftDrive.getEncoder().getPosition()-previousPositionLeft)+(rightDrive.getEncoder().getPosition()-previousPositionRight))/2;
            expectedStep *= Math.PI*WheelDiam;
            confidence = MAX_CONFIDENCE - Math.abs(leftDrive.getEncoder().getVelocity() - rightDrive.getEncoder().getVelocity())*VELOCITY_TO_CONFIDENCE;
        }

        previousPositionLeft = leftDrive.getEncoder().getPosition();
        previousPositionRight = rightDrive.getEncoder().getPosition();

        double DegreesToWall = 90;//TODO figure out how to calculate this

        Gaussian prediction = new Gaussian(expectedStep*Math.sin(DegreesToWall), 1/confidence);

        return prediction;
    }

    @Override
    public double sensorValue() {
        return ultrasonicSensor.getVoltage()*voltageToInches;
    }

}