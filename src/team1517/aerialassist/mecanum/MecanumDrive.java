/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package team1517.aerialassist.mecanum;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

/**
 *
 * @author Zoraver
 */
public class MecanumDrive {
    
    CANJaguar aF, aB, bF, bB;
    
    public MecanumDrive(CANJaguar aFront, CANJaguar aBack, CANJaguar bFront, CANJaguar bBack)
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
            
            magnitude = Math.sqrt(x * x + y * y);
            theta = MathUtils.atan(y / x);
            
            if(x < 0)
            {
                theta = theta + Math.PI;
            }
            
            A = Math.sqrt(2) * Math.sin(theta - 3 * Math.PI / 4);
            B = Math.sqrt(2) * Math.cos(theta + Math.PI / 4);
            
            if(A > 1)
            {
                A = 1;
            }
            else if(A < -1) 
            {
                A = -1;
            }
            
            if(B > 1) 
            {
                B = 1;
            }
            else if(B < -1) 
            {
                B = -1;
            }
            
            A = A * magnitude;
            B = B * magnitude;
            
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
            
            try
            {
                aF.setX((A + T) / highestValue); 
                aB.setX((A - T) / highestValue); 
                bF.setX((B - T) / highestValue); 
                bB.setX((B + T) / highestValue); 
            }
            catch(CANTimeoutException ex)
            {
                ex.printStackTrace();
                return false;
            }
        return true;
    }
}
