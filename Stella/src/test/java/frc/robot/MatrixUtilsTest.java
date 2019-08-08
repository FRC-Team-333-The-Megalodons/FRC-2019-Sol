package frc.robot;

//import static org.junit.Assert.assertEquals;

import static org.junit.Assert.*;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import org.junit.Test;

import frc.robot.kalmanfilter.multivariate.Gaussians;
import frc.robot.kalmanfilter.multivariate.MatrixUtils;

public class MatrixUtilsTest{

    @Test
    public void testDeterminant(){
        double toTest[][] = {{1, 0, 2, -1}, 
                        {3, 0, 0, 5}, 
                        {2, 1, 4, -3}, 
                        {1, 0, 5, 0}}; 
        
        assertEquals(30.0, MatrixUtils.determinant(toTest), 0.0);
    }

    @Test
    public void testMask(){
        double toTest[][] = {{1, 0, 2, -1}, 
                        {3, 0, 0, 5}, 
                        {2, 1, 4, -3}, 
                        {1, 0, 5, 0}}; 
        double expectedResult[][] = { 
                        { 0, 0, 5}, 
                        { 1, 4, -3}, 
                        { 0, 5, 0}}; 

        assertArrayEquals(expectedResult, MatrixUtils.mask(toTest, 0 ,0));
    }

    @Test
    public void dotTest(){
        double [][] x = {{1,2}, {3,4}};
        double [] y = {10, 4.5};
        double [] expectedResult = {19,48};
        assertArrayEquals(expectedResult, MatrixUtils.dot(x,y));
    }

    @Test
    public void dotProduct(){
        double [] x = {8,57};
        double [] y = {25,258};
        assertEquals(14906, MatrixUtils.dotProd(x,y), 0.0);
    }

    @Test
    public void transTest(){
        double [][] y = {{1}, {3},{7}};
        double [][] result = {{1,3,7}};
        assertArrayEquals(result, MatrixUtils.transpose(y));
    }

    @Test
    public void CovarianceMatrixTest(){
        double [] input = {1,4,3,4,5};
        double [] dummyMeans ={0,0,0,0,0};
        String [] names = {"one","two","three","four", "five"};

        Gaussians G = new Gaussians(dummyMeans, input, names);

        double [][] expectedResult = {{1,0,0,0,0},
                                    {0,4,0,0,0},
                                    {0,0,3,0,0},
                                    {0,0,0,4,0},
                                    {0,0,0,0,5}};

        assertArrayEquals(expectedResult, G.getCovariance(), G.toString());
    }
}