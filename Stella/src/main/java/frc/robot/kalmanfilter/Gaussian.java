package frc.robot.kalmanfilter;
public class Gaussian {
    private double mean, variance;


    public Gaussian(double mean, double variance){
        this.mean = mean;
        this.variance = variance;
    }

    public double getMean(){
        return mean;
    }

    public double getVariance(){
        return variance;
    }

    /**
     * 
     * @param one the first gaussian to sum
     * @param two the second gaussian to sum
     * @return the sum of the two passed gaussians
     */
    public static Gaussian add(Gaussian one, Gaussian two){
        Gaussian sum = new Gaussian(one.getMean() + two.getMean(), one.getVariance()+two.getVariance());
        return sum;
    }

    /**
     * 
     * @param one the first gaussian to multiply
     * @param two the second gaussian to multiply
     * @return the product of the two passed gaussians
     */
    public static Gaussian multiply(Gaussian one, Gaussian two){
        return new Gaussian(multiplyMean(one, two), multiplyVariance(one, two));
    }

    /**
     * helper method for multiplying gaussians
     * @param one the first gaussian to multiply
     * @param two the first gaussian to multiply
     * @return the mean for the product of one and two
     */
    private static double multiplyMean(Gaussian one, Gaussian two){
        return(one.getVariance()*two.getMean()+two.getVariance()+one.getMean())/(one.getVariance()+two.getVariance());
    }

    /**
     * helper method for multiplying gaussians
     * @param one the first gaussian to multiply
     * @param two the first gaussian to multiply
     * @return the variance for the product of one and two
     */
    private static double multiplyVariance(Gaussian one, Gaussian two){
        return(one.getVariance()*two.getVariance())/(one.getVariance()+two.getVariance());
    }

    /**
     * 
     * how likey is it that the system is in the state with a value of x
     * 
     * @param x value to check
     * @return likelyhood that the system is in the state with a value of x
     */
    public double getLikelyhood(double x){
        return (1/Math.pow(2*Math.PI*variance, 1/2))*Math.pow(Math.E, Math.pow(x-mean, 2)/2*variance);
        //gaussian formula
    }
}