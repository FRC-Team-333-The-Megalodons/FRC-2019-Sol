package frc.robot.kalmanfilter.multivariate;

import java.util.HashMap;
import frc.robot.kalmanfilter.multivariate.MatrixUtils;

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

    @Override
    public String toString(){
        return "this is an "+ size + "D kalmnan filter" + 
        "\nThe means are :" + means.toString() + 
        "\nThe means represnt :" + meanNames.toString()+ 
        "\nThe covariances are :" + covarianceMatrix.toString();
    }

    public HashMap<String , Double> getMeans(){
        
        HashMap<String , Double> meansWithNames = new HashMap<>(means.length+1, 1);

        for (int i = 0; i < means.length; i++) {
            meansWithNames.put(meanNames[i],means[i]);
        }

        return meansWithNames;
    }

    public int getSize() {
        return this.size;
    }

    /*public double getLikelyhood(double [] values){
        double[] [] verticalmeans = new double [0][means.length];
        verticalmeans[0] = MatrixUtils.subtract(values, means);

        double normalizationTerm = 1/Math.pow(Math.pow(2*Math.PI, values.length)*MatrixUtils.determinant(covarianceMatrix), 1/2);

        return Math.pow(Math.E, MatrixUtils.multiply(MatrixUtils.dot(MatrixUtils.dot(MatrixUtils.transpose(verticalmeans), verticalmeans), MatrixUtils.raise(covarianceMatrix, -1)), -1/2));
    } 
    IDK how this formula works butr it's not strictly nessary sooooooo
    */

    
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