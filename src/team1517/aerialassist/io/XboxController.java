/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package team1517.aerialassist.io;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Class representing the Xbox controller.
 * @author Zoraver
 */
public class XboxController {
    
    Joystick xb;
    
    /**
     * Constructs the XboxController object with the USB port number.
     * @param port The number of the USB port in which the Xbox controller is connected.
     */
    public XboxController(int port)
    {
        xb = new Joystick(port);
    }
    
    public boolean getButtonA()
    {
        return xb.getRawButton(1);
    }
    
    public boolean getButtonB()
    {
        return xb.getRawButton(2);
    }
    
    public boolean getButtonX()
    {
        return xb.getRawButton(3);
    }
    
    public boolean getButtonY()
    {
        return xb.getRawButton(4);
    }
    
    public boolean getLeftTriggerButton()
    {
        return xb.getRawButton(5);
    }
    
    public boolean getRightTriggerButton()
    {
        return xb.getRawButton(6);
    }
    
    public boolean getBackButton()
    {
        return xb.getRawButton(7);
    }
    
    public boolean getStartButton()
    {
        return xb.getRawButton(8);
    }
    
    public boolean getLeftJoystickButton()
    {
        return xb.getRawButton(9);
    }
    
    public boolean getRightJoystickButton()
    {
        return xb.getRawButton(10);
    }
    
    public double getLeftJoystickX()
    {
        return xb.getRawAxis(1);
    }
    
    public double getLeftJoystickY()
    {
        return xb.getRawAxis(2);
    }
    
    public double getRightJoystickX()
    {
        return xb.getRawAxis(4);
    }
    
    public double getRightJoystickY()
    {
        return xb.getRawAxis(5);
    }
    
    public double getAnalogTriggers()
    {
        return xb.getRawAxis(3);
    }
    
    public double getDpadX()
    {
        return xb.getRawAxis(6);
    }
    
    public double getLeftJoystickMagnitude()
    {
        return Math.sqrt(getLeftJoystickX() * getLeftJoystickX() + getLeftJoystickY() * getLeftJoystickY());
    }
    
    public double getRightJoystickMagnitude()
    {
        return Math.sqrt(getRightJoystickX() * getRightJoystickX() + getRightJoystickY() * getRightJoystickY());
    }
    
    public double getLeftJoystickAngle()
    {
        double angle = MathUtils.atan(getLeftJoystickY() / getLeftJoystickX());
        
        if(getLeftJoystickX() < 0)
        {
            return angle + Math.PI;
        }
        else return angle;
    }
    
    public double getRightJoystickAngle()
    {
        double angle = MathUtils.atan(getRightJoystickY() / getRightJoystickX());
        
        if(getRightJoystickX() < 0)
        {
            return angle + Math.PI;
        }
        else return angle;
    }
}
