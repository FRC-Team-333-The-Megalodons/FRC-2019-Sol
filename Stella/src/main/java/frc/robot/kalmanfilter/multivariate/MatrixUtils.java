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

    public static double determinant(double [] [] X) {

        //exit condition use determinant formula for a 3x3 matrix
        if( X.length == 2){
            return X[0][0]*X[1][1] - X[0][1]*X[1][0];
        }

        //keep track of the + - + - pattern for computing determinants
        boolean negative = false;

        int det = 0;

        //go through each collum and calculate the determinant recursively
        for(int i =0; i < X[0].length; i++){

                //sum up the determinant using the + - pattern and the "masking" technique
                det += (negative?-1:1)*(X[0][i]*determinant(mask(X, 0, i)));

                //to0ggle the sign
                negative = !negative;
        }

        //pass the determinant up the chain
        return det;
            
    }


    public static double[][] mask(double[][] toMask, int row, int collum) {

        //our result will be an array 1 smaller than the input in both dimensions
        double [][] result = new double[toMask.length-1][toMask[0].length-1];

        //set a flag to keep track of if we've passed the row to skip
        boolean iOffset = false;

        for (int i = 0; i < toMask.length; i++) {

            //check if we're at the row we need to skip
            if(i == row){

                //skip this row and toggle ioffset to declare that we've skipped a row
                i++;
                iOffset = true;

                //if we're skipping the last row break
                if(i >= toMask.length){
                    break;
                }
            }

            //everything here follows what we do horrizontally just does it vertically
            boolean jOffset = false;
            for (int j = 0; j < toMask[0].length; j++) {

                if(j == collum){
                    j++;
                    jOffset = true;
                    if(j >= toMask[0].length){
                        break;
                    }
                }

                //place the element at i,j from the origonal array into the masked array 
                //offest the position accordin to if we've passed the row/collum
                result[i + (iOffset?-1:0)][j + (jOffset?-1:0)] = toMask[i][j];
            }
        }
    
        return result;
    }

    public static double[] getCollum(double[][] b, int collumNum) {
        double [] collum = new double [b.length];

        for (int i = 0; i < b.length; i++) {
            collum[i] = b[i][collumNum];
        }

        return collum;
    }

    public static double[][] transpose(double[][] A){
        double[][] x = A;
        for (int i = 0; i < x[0].length; i++) {
        x[i] = getCollum(A, i);
        }
        return x;
    }

    public static double[] subtract(double [] A, double [] B){
        double [] x =  A;
        for (int i = 0; i < x.length; i++) {
            x[i] = A[i]-B[i];
        }
        return x;
    }

    public static double[][] multiply(double [][] A, double X){
        for (int i = 0; i < A.length; i++) {
            A[i] = multiply(A[i], X);
        }
        return A;
    }
    

    public static double[] multiply(double [] A, double X){
        for (int i = 0; i < A.length; i++) {
            A[i] *= X;
        }
        return A;
    }

    public static double[] raise(double [] A, double X){
        for (int i = 0; i < A.length; i++) {
            A[i] = Math.pow(A[i], X);
        }
        return A;
    }

    public static double[][] raise(double [][] A, double X){
        for (int i = 0; i < A.length; i++) {
            A[i] = raise(A[i], X);
        }
        return A;
    }
}