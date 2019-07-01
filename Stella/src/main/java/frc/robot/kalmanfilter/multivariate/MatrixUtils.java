package frc.robot.kalmanfilter.multivariate;

public class MatrixUtils {
    
    public static double [][] dot(double [][] A, double [] [] B){
        double [][] output = new double [Math.min(A.length, B.length)][Math.min(A[0].length, B[0].length)];

        for (int i = 0; i < output.length; i++) {
            for (int j = 0; i < output[0].length; j++) {
                output[i][j] = dotProd(A[i], getCollum(B , j));
            }
        }

        return output;
    }

    public static double dotProd(double[] A, double[] B) {
        double prod = 0;

        if(A.length != B.length){
            return -1;
        }

        for (int i = 0; i < A.length; i++) {
            prod += A[i]*B[i];
        }
        
        return prod;
    }

    public static double[] getCollum(double[][] b, int j) {
        double [] collum = new double [b.length];

        for (int i = 0; i < b.length; i++) {
            collum[i] = b[i][j];
        }

        return collum;
    }
}