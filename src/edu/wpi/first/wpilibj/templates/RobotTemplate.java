/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.can.CANTimeoutException;

import team1517.aerialassist.mecanum.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    CANJaguar aF, aB, bF, bB, winchMotor;
    Victor rotRod1, rotRod2, angle;
    Joystick xyStick, steerStick, auxStick;
    DriverStationLCD lcd;
    MecanumDrive mDrive;
    
    public RobotTemplate()
    {
        rotRod1 = new Victor(1);
        rotRod2 = new Victor(2);
        angle = new Victor(3);
        
        xyStick = new Joystick(1);
        steerStick = new Joystick(2);
        auxStick = new Joystick(3);
        
        lcd = DriverStationLCD.getInstance();
        
        initCANJaguars();
        
        mDrive = new MecanumDrive(aF, aB, bF, bB);
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        boolean exceptionFree;
        
        while(isOperatorControl() && isEnabled())
        {
            exceptionFree = mDrive.drive(filterJoystickInput(xyStick.getX()), filterJoystickInput(xyStick.getY()), steerStick.getTwist());
            if(!exceptionFree || getCANJaguarsPowerCycled())
                initCANJaguars();
            
            lcd.println(DriverStationLCD.Line.kUser1, 1, " " + steerStick.getTwist());
            lcd.updateLCD();
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
    
    
    private boolean getHotGoal()
    {
        //todo stub.
        return true;
    }
    
    /**
     * Used to initialize the CANJaguars.  It can also be called to reinitialize them if an exception is thrown.
     * @return Success
     */
    private boolean initCANJaguars()
    {
        try
        {
            aF = null;
            bF = null;
            aB = null;
            bB = null;
            winchMotor = null;
            
            aF = new CANJaguar(1);
            bF = new CANJaguar(2);
            aB = new CANJaguar(3);
            bB = new CANJaguar(4);
            //winchMotor = new CANJaguar(5);
            
            aF.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            bF.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            aB.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            bB.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            //winchMotor.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            
            aF.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            bF.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            aB.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            bB.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            //winchMotor.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            
            aF.configEncoderCodesPerRev(100);
            bF.configEncoderCodesPerRev(100);
            aB.configEncoderCodesPerRev(100);
            bB.configEncoderCodesPerRev(100);
            //winchMotor.configEncoderCodesPerRev(100);
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            return false;
        }
        return true;
    }
    
    private boolean getCANJaguarsPowerCycled()
    {
        try
        {
            if(aF.getPowerCycled() || aB.getPowerCycled() || bF.getPowerCycled() || bB.getPowerCycled())
            {
                return true;
            }
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            return true;
        }
        return false;
    }
    
    double filterJoystickInput(double joystickValue)
    {
        if(Math.abs(joystickValue) > 0.1)
        {
            return joystickValue;
        }
        else 
        {
            if(steerStick.getTwist() != 0)
            {
                return 0.0000000000001;
            }
            else
            {
            return 0;
            }
        }
    }
}
