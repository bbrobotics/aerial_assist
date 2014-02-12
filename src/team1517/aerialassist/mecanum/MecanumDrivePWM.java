/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package team1517.aerialassist.mecanum;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Jaguar;

/**
 *
 * @author Zoraver
 */
public class MecanumDrivePWM {
    
    Jaguar aF, aB, bF, bB;
    
    public MecanumDrivePWM(Jaguar aFront, Jaguar aBack, Jaguar bFront, Jaguar bBack)
    {
        aF = aFront;
        aB = aBack;
        bF = bFront;
        bB = bBack;
    }
    
    public boolean drive(double mX, double mY, double twist)
    {   
        double x, y, magnitude, theta, highestValue, A, B, T;
            
            x = mX;
            y = mY;
            T = twist;
            
            magnitude = Math.sqrt(x * x + y * y);//Calculates the magnitude of the output vector.
            theta = MathUtils.atan(y / x);//Calculates the direction of the output vector.
            
            if(x < 0)//Corrects the quadrent of theta for the value of X.
            {
                theta = theta + Math.PI;
            }
            
            A = Math.sqrt(2) * Math.sin(theta - 3 * Math.PI / 4);//Sets diagonal A to the value for theta of mechanum graph.
            B = Math.sqrt(2) * Math.cos(theta + Math.PI / 4);//Sets diagonal B to the value for theta of the mechanum equation.
            
            if(A > 1)//Scales A to 1 if it is higher than one.
            {
                A = 1;
            }
            else if(A < -1)//Scales A to -1 if it is less than one. 
            {
                A = -1;
            }
            
            if(B > 1)//Scales B to 1 if it is higher than one. 
            {
                B = 1;
            }
            else if(B < -1)//Scales B to -1 if it is less than one. 
            {
                B = -1;
            }
            
            A = A * magnitude;//Scales A and B to their actual values.
            B = B * magnitude;
            
            //Scales the outputs by the value of the highest output, if it ls higher than 1.
            if(Math.abs(A + T) > 1 || Math.abs(A - T) > 1 || Math.abs(B + T) > 1 || Math.abs(B - T) > 1)
            {
                highestValue = Math.abs(A + T);
                
                if(Math.abs(A - T) > highestValue)
                {
                    highestValue = Math.abs(A - T);
                }
                
                if(Math.abs(B + T) > highestValue)
                {
                    highestValue = Math.abs(B + T);
                }
                
                if(Math.abs(B - T) > highestValue)
                {
                    highestValue = Math.abs(B - T);
                }
            }
            else
            {
                highestValue = 1.0;
            }
            
            aF.set((A + T) / highestValue); 
            aB.set((A - T) / highestValue); 
            bF.set((B - T) / highestValue); 
            bB.set((B + T) / highestValue); 
            
        return true;//Returns true if successful.
    }
}
    
    