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
    final double mP = 1, mI = 0, mD = 0;//All final variables will have to be tweaked through testing.
    final int MAX_RPM = 100;
    
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
            
        try 
        {
            //Sets the control mode of the CANJaguars to percentVbus.
            if(aF.getControlMode() != CANJaguar.ControlMode.kPercentVbus)
            {
                aF.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
                aB.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
                bF.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
                bB.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            }
        } 
        catch (CANTimeoutException ex) 
        {
            ex.printStackTrace();
            return false;
        }
        
            x = mX;
            y = mY;
            T = twist;
            
            magnitude = Math.sqrt(x * x + y * y);//Calculates the magnitude of the output vector.
            theta = MathUtils.atan(y / x);//Calculates the direction of the output vector.
            
            if(x < 0)
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
        return true;//Returns true if successful.
    }
    
    public boolean pidDrive(double mX, double mY, double twist)
    {
        try 
        {
            if(aF.getControlMode() != CANJaguar.ControlMode.kSpeed)
            {
                aF.changeControlMode(CANJaguar.ControlMode.kSpeed);
                aB.changeControlMode(CANJaguar.ControlMode.kSpeed);
                bF.changeControlMode(CANJaguar.ControlMode.kSpeed);
                bB.changeControlMode(CANJaguar.ControlMode.kSpeed);
            }
            
            aF.setPID(mP, mI, mD);
            aB.setPID(mP, mI, mD);
            bF.setPID(mP, mI, mD);
            bB.setPID(mP, mI, mD);
            
            
        } 
        catch (CANTimeoutException ex) 
        {
            ex.printStackTrace();
            return false;
        }
        
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
            aF.setX(MAX_RPM * ((A + T) / highestValue)); 
            aB.setX(MAX_RPM * ((A - T) / highestValue)); 
            bF.setX(MAX_RPM * ((B - T) / highestValue)); 
            bB.setX(MAX_RPM * ((B + T) / highestValue)); 
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            return false;
        }
        return true;
    }
}
