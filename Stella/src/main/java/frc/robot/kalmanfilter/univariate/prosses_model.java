package frc.robot.kalmanfilter.univariate;

abstract class prosses_model{

    double variance;
    
    /**calculate a change in the read value given a time step t*
     * 
     * @param t time step t
     * @return  a gausian with the mean being the predicted change in value
    */
    abstract public Gaussian update(double t);
    
    /**
     * 
     * @return the current RAW sensor value
     */
    abstract public double sensorValue();

}