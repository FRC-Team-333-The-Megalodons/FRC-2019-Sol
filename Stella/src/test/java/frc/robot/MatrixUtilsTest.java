package frc.robot;

//import static org.junit.Assert.assertEquals;

import static org.junit.Assert.*;

import org.hamcrest.*;
import org.junit.Test;
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

}