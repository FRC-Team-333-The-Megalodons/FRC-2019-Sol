package frc.robot.kalmanfilter.multivariate;


public class Gaussians{

    String [] meanNames;
    double [] means;
    double [] [] covarianceMatrix;
    int size;

    public Gaussians(double [] initalMeans, double [][] initalCovariance, String [] meanNames){
        
        this.meanNames= meanNames;
        this.means = initalMeans;
        this.covarianceMatrix = initalCovariance;

        this.size = initalCovariance.length;
    }

    public Gaussians(double [] initalMeans, double [] initalVariance, String [] meanNames){
        
        this.meanNames= meanNames;
        this.means = initalMeans;


        /**
         * fill a square 2d array as follows based on a 1d array
         * 
         *  [x,y,z,a,b]
         * 
         *  [[x,0,0,0,0],
         *   [0,y,0,0,0],
         *   [0,0,z,0,0],
         *   [0,0,0,a,0],
         *   [0,0,0,0,b]]
         * 
         * this is for situations where the inital corralation is unknown
         */

        this.covarianceMatrix = new double[initalVariance.length][initalVariance.length];

        for (int i = 0; i < initalVariance.length; i++) {
           for (int j = 0; j < initalVariance.length; i++) {
               if(i == j){
                   this.covarianceMatrix[i][j] = initalVariance[i];
               }else{
                   this.covarianceMatrix[i][j] = 0;
               }
           } 
        }
        
    }

    public double getMean(String meanName) throws NameNotFoundException {
        
        for (int i = 0; i < meanNames.length; i++) {
            if(meanName.equals(meanNames[i])){
                return means[i];
            }
        }
         
        throw new NameNotFoundException(meanName);

    }

    public String toString(){
        return "this is an "+ size + "D kalmnan filter" + 
        "\nThe means are :" + means.toString() + 
        "\nThe values represnt are :" + meanNames.toString()+ 
        "\nThe covariances are :" + covarianceMatrix.toString();
    }
}

class NameNotFoundException extends Exception {

    private static final long serialVersionUID = 1L;

    String name;

    public NameNotFoundException (String name){
        this.name = name;
    }

    public String toString(){
        return "Could not find mean with name : " + name;
    }
}