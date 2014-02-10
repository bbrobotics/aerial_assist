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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

import team1517.aerialassist.mecanum.MecanumDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    
    boolean catapultArmed = false;
    final int AREA_MINIMUM = 100;
    double tiltValue = 0.5, rotValue = 0.85;
    
    AxisCamera camera;
    CriteriaCollection cc;
    CANJaguar aF, aB, bF, bB;
    DigitalInput armedSwitch;
    Victor rotRod1, rotRod2, angle1;
    Talon winchMotor;
    Servo tiltServo, rotServo;
    Joystick xyStick, steerStick, auxStick;
    DriverStationLCD lcd;
    MecanumDrive mDrive;
    
    public RobotTemplate()
    {
        camera = AxisCamera.getInstance();
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 215472, false);
        
        armedSwitch = new DigitalInput(1);
        
        rotRod1 = new Victor(1);
        rotRod2 = new Victor(2);
        angle1 = new Victor(3);
        
        winchMotor = new Talon(4);
        
        tiltServo = new Servo(5);
        rotServo = new Servo(6);
        
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
        Timer.delay(0.7);//Delays a amount of time in order for the hot goal vision targets to rotate into position.
        boolean isHotGoalStarting = getHotGoal();
        try
        {
            while(aF.getPosition() < 0.7)
            {
                mDrive.drive(0, 1, 0);
            }
            mDrive.drive(0, 0, 0);   
        }
        catch(CANTimeoutException ex)
        {
            ex.printStackTrace();
            initCANJaguars();
        }
        if(!isHotGoalStarting)
        {
            Timer.delay(4);
        }
        //Shoot.        
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        boolean exceptionFree;
        
        while(isOperatorControl() && isEnabled())
        {
            /*
             * Controls the drive base and also handles exceptions.
             */
            exceptionFree = mDrive.drive(filterJoystickInput(xyStick.getX()), filterJoystickInput(xyStick.getY()), filterJoystickInput(xyStick.getTwist()));
            if(!exceptionFree || getCANJaguarsPowerCycled())
            {
                initCANJaguars();
            }
            
            angle1.set(auxStick.getY() * 0.7);
            
            if(auxStick.getRawButton(3))
            {
                rotRod1.set(-0.5);
                rotRod2.set(0.5);
            }
            else if(auxStick.getRawButton(5))
            {
                rotRod1.set(0.5);
                rotRod2.set(-0.5);
            }
            else
            {
                rotRod1.set(0);
                rotRod2.set(0);
            }
            
            tiltServo.set(tiltValue);
            rotServo.set(rotValue);
            
            if(auxStick.getRawAxis(6) > 0 && tiltValue <= 0.95)
            {
                tiltValue = tiltValue + 0.05;
            }
            else if(auxStick.getRawAxis(6) < 0 && tiltValue >= 0.05)
            {
                tiltValue = tiltValue - 0.05;
            }
            
            if(auxStick.getRawAxis(5) > 0 && rotValue <= 0.95)
            {
                rotValue = rotValue + 0.05;
            }
            else if(auxStick.getRawAxis(5) < 0 && rotValue >= 0.05)
            {
                rotValue = rotValue - 0.05;
            }
            
            lcd.println(DriverStationLCD.Line.kUser1, 1, "tilt " + tiltValue + "       ");
            lcd.println(DriverStationLCD.Line.kUser2, 1, "rot " + rotValue + "       ");
            
            Timer.delay(0.1);
            lcd.updateLCD();
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() 
    {
        while(isTest() && isEnabled())
        {
            if(auxStick.getRawButton(1))
            {
                lcd.println(DriverStationLCD.Line.kUser1, 1, " " + getHotGoal());
            }
            
            if(auxStick.getRawButton(2))
            {
                lcd.println(DriverStationLCD.Line.kUser2, 1, " " + getVisionDistance());
            }
            
            lcd.updateLCD();
        }
    }
    
    /**
     * Moves the catapult into armed position.
     */
    void armCatapult()
    {
        if(!catapultArmed)
        {
            while(!armedSwitch.get())
            {
                winchMotor.set(0.7);
            }
            winchMotor.set(0);
            catapultArmed = true;
        }
    }
    
    /**
     * Fires the catapult.
     */
    void fireCataput()
    {
        if(catapultArmed)
        {
            Timer timer = new Timer();
            timer.start();
            
            while(timer.get() < 0.25)//Adjust as nessisary.
            {
                winchMotor.set(0.7);
            }
            
            
            winchMotor.set(0);
            catapultArmed = false;
            timer.stop();
        }
    }
    
    private boolean getHotGoal()
    {
        try {
            ColorImage image = camera.getImage();
            BinaryImage thresholdImage = image.thresholdHSV(80, 140, 165, 255, 200, 255);
            image.free();
            BinaryImage hulledImage = thresholdImage.convexHull(false);
            thresholdImage.free();
            
            
            if(hulledImage.getNumberParticles() > 0)
            {
                lcd.println(DriverStationLCD.Line.kUser2,1, "" + hulledImage.getNumberParticles());
                lcd.updateLCD();
                ParticleAnalysisReport report;
                for(int i = 0; i < hulledImage.getNumberParticles(); i++)
                {
                   report = hulledImage.getParticleAnalysisReport(i);
                   if((report.boundingRectHeight / report.boundingRectWidth) < 1)
                   {
                       return true;
                   }
                }
                report = null;
            }
            hulledImage.free();
        } catch (AxisCameraException ex) {
            ex.printStackTrace();
            return false;
        } catch (NIVisionException ex) {
            ex.printStackTrace();
            return false;
        }
        return false;
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
            
            aF = new CANJaguar(1);
            bF = new CANJaguar(2);
            aB = new CANJaguar(3);
            bB = new CANJaguar(4);
            
            aF.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            bF.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            aB.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            bB.changeControlMode(CANJaguar.ControlMode.kPercentVbus);
            
            aF.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            bF.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            aB.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            bB.setSpeedReference(CANJaguar.SpeedReference.kQuadEncoder);
            
            aF.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            bF.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            aB.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            bB.setPositionReference(CANJaguar.PositionReference.kQuadEncoder);
            
            aF.configEncoderCodesPerRev(100);
            bF.configEncoderCodesPerRev(100);
            aB.configEncoderCodesPerRev(100);
            bB.configEncoderCodesPerRev(100);
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
            return (joystickValue * joystickValue * joystickValue);
        }
        else 
        {
            if(xyStick.getTwist() != 0)
            {
                return 0.0000000000001;
            }
            else
            {
            return 0;
            }
        }
    }
    
    double getVisionDistance()
    {
        try 
        {
            ColorImage image = camera.getImage();
            BinaryImage thresholdImage = image.thresholdHSV(80, 140, 165, 255, 200, 255);
            image.free();
            BinaryImage hulledImage = thresholdImage.convexHull(false);
            thresholdImage.free();
            
            if(hulledImage.getNumberParticles() > 0)
            {
                ParticleAnalysisReport report;
                for(int i = 0; i < hulledImage.getNumberParticles(); i++)
                {
                    report = hulledImage.getParticleAnalysisReport(i);
                    if(report.boundingRectWidth / report.boundingRectHeight < 1) //1 can be reduced.
                    {
                        //do distance calculations.
                        //return distance.
                        return report.center_mass_y_normalized + 1;
                    }
                }
            }
           
        } 
        catch (AxisCameraException ex) 
        {
            ex.printStackTrace();
            return -2;
        }
        catch (NIVisionException ex)
        {
            ex.printStackTrace();
            return -3;
        }
        
        return -1;
    }
}
