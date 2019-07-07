package frc.robot.kalmanfilter.univariate;

public class uniVariateFilter {

    private double sensorVariance;
    private prosses_model p;
    private Gaussian value;

    /**
     * 
     * @param p prosses model to predict values
     * @param initalCondition first value to use in prediction step
     * @param sensorVariance inverse of confidence in sensor
     */
    public uniVariateFilter(prosses_model p, double initalCondition, double sensorVariance){
        this.p = p;
        this.value = new Gaussian(initalCondition, sensorVariance);
    }
    /**
     * 
     * update the estimate based on a passed sensor value and time step t
     * 
     * @param t time step
     * @param sensorValue sensor value
     * @return gausian with the mean being the prdicted value and variance being the inverse of confidence in the mean
     */
    public Gaussian update(double t, double sensorValue){
        Gaussian preditction = Gaussian.add(value, p.update(t));
        Gaussian sensorGausian = new Gaussian(sensorValue, sensorVariance);
        value = Gaussian.multiply(preditction, sensorGausian);
        return value;
    }

    /**
     * 
     * update the estimate based on the sensor from the prosses model and time step t
     * 
     * @param t time step
     * @return gausian with the mean being the prdicted value and variance being the inverse of confidence in the mean
     */
    public Gaussian update(double t){
        Gaussian preditction = Gaussian.add(value, p.update(t));
        Gaussian sensorGausian = new Gaussian(p.sensorValue(), sensorVariance);
        value = Gaussian.multiply(preditction, sensorGausian);
        return value;
    }

    /**
     * 
     * @param t time step
     * @param sensorValue current raw sensor reading
     * @return the best estimate of the filter
     */
    public double updateAndGetEstimate(double t, double sensorValue){
        return update(t, sensorValue).getMean();
    }

    /**
     * 
     * update the estimate based on the sensor from the prosses model and time step t
     * 
     * @param t time step
     * @param sensorValue current raw sensor reading
     * @return the best estimate of the filter
     */
    public double updateAndGetEstimate(double t){
        return update(t).getMean();
    }

    /**
     * @return the mean value WITHOUT UPDATING
     */
    public double getEstimate(){
        return value.getMean();
    }

    /**
     * 
     * @return latest Gausian WITHOUT UPDATING
     */
    public Gaussian getValue() {
        return value;
    }

    public void setSensorVariance(double sensorVariance){
        this.sensorVariance = sensorVariance;
    }

    public void setProssesModel(prosses_model p){
        this.p = p;
    }

}