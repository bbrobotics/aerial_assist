/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Jaguar;
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
import team1517.aerialassist.io.DriverLCD;

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
    double tiltValue = 0.5, rotValue = 0.85, winchPower = -1;
    
    AxisCamera camera;
    CriteriaCollection cc;
    CANJaguar aF, aB, bF, bB;
    DriverStation driverStation;
    DigitalInput armedSwitch;
    Victor rotRod1, rotRod2, angle1, rot2rod2, rot2rot2, angle2;
    Talon winchMotor;
    Servo tiltServo, rotServo;
    Joystick xyStick, steerStick, auxStick;
    DriverLCD lcd;
    MecanumDrive mDrive;
    
    public RobotTemplate()
    {
        //camera = AxisCamera.getInstance();
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 215472, false);
        
        armedSwitch = new DigitalInput(1);
        
        driverStation = DriverStation.getInstance();
        
        rotRod1 = new Victor(8);
        rotRod2 = new Victor(9);
        angle1 = new Victor(3);
        
//        rot2Rod1 = new Victor()
//        rot2Rod2 = new Victor()
//        angle2 = new Victor()
//        
        
        winchMotor = new Talon(4);
        
        tiltServo = new Servo(5);
        rotServo = new Servo(6);
        
        xyStick = new Joystick(1);
        steerStick = new Joystick(2);
        auxStick = new Joystick(3);
        
        lcd = new DriverLCD();
        
        initCANJaguars();
    }
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        boolean isLowGoal = driverStation.getDigitalIn(1);
        Timer aTimer = new Timer();
        aTimer.start();
        tiltServo.set(0.5);
        rotServo.set(0.5);
        
        if(isLowGoal)
        {
           //Timer.delay(0.7);//Delays a amount of time in order for the hot goal vision targets to rotate into position.
            boolean isHotGoalStarting = true;//getHotGoal();
            try
            {
                if(!isHotGoalStarting)
                {
                    Timer.delay(2);
                }
                while(Math.abs(bF.getPosition()) < 10.18 && aTimer.get() < 10)
                {
                    mDrive.drive(0, -0.7, 0);
                    lcd.println(1, 1, "" + aF.getPosition());
                    lcd.updateLCD();
                }
                mDrive.drive(0, 0, 0);
            }
            catch(CANTimeoutException ex)
            {
                ex.printStackTrace();
                initCANJaguars();
            } 
        }
        else
        {
            aTimer.reset();
            while(aTimer.get() < 0.2)
            {
                angle1.set(0.3);
            }
            angle1.set(0);
            
            /*try
            {
                while(Math.abs(bF.getPosition()) < 8.12 && aTimer.get() < 10)
                {
                    mDrive.drive(0, -0.7, 0);
                }
                mDrive.drive(0, 0, 0);
                if(!armedSwitch.get())
                {
                    while(!armedSwitch.get())
                    {
                        armCatapult();
                    }
                }
                fireCatapult();
            }
            catch(CANTimeoutException ex)
            {
                ex.printStackTrace();
                initCANJaguars();
            }*/
        }
        
        aTimer.stop();
       
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        boolean exceptionFree = true;
        double x = 0, y = 0, t = 0;
        
        while(isOperatorControl() && isEnabled())
        {
            if(xyStick.getRawButton(1))
            {
                try
                {
                    aF.setX(xyStick.getTwist());
                    bB.setX(xyStick.getTwist());
                    aB.setX(-1 * xyStick.getTwist());
                    bF.setX(-1 * xyStick.getTwist());
                    exceptionFree = true;
                }
                catch(CANTimeoutException ex)
                {
                    ex.printStackTrace();
                    exceptionFree = false;
                }
            }
            else
            {
                /*
                 * Controls the drive base and also handles exceptions.
                 */
                x = filterJoystickInput(steerStick.getX());
                y = filterJoystickInput(xyStick.getY());
                t = filterJoystickInput(xyStick.getTwist());

                exceptionFree = tDrive(x, y, t); 
            }
            
            if(!exceptionFree)
            {
                initCANJaguars();
            }   
            
            /*
             * Sets the output to the angle motor of the rot rods to the value of the y axis of the auxStick scaled by a factor of 0.7.
             */
            angle1.set(auxStick.getY());

            /*
             * Controls the rotation of the rot rods.
             */
            if(auxStick.getRawButton(3))
            {
                rotRod1.set(-0.7); 
                rotRod2.set(0.7);
            }
            else if(auxStick.getRawButton(5))
            {
                rotRod1.set(0.7);
                rotRod2.set(-0.7);
            }
            else
            {
                rotRod1.set(0);
                rotRod2.set(0);
            }
            
            /*
             * Manual control of the catapult winch.
             */
            if(auxStick.getRawButton(2))
            {
                winchMotor.set(-1);
            }
            else if(auxStick.getRawButton(4))
            {
                armCatapult();
            }
            else if(auxStick.getRawButton(6))
            {
                fireCatapult();
            }
            else
            {
                winchMotor.set(0);
            }

            if(auxStick.getRawAxis(6) > 0 && winchPower <= 0.95)
            {
                winchPower = winchPower + 0.05;
            }
            else if(auxStick.getRawAxis(6) < 0 && winchPower >= -0.95)
            {
                winchPower = winchPower - 0.05;
            }
            
            /*
             * Sets the output values of the camera axis servos.
             */
            tiltServo.set(tiltValue);
            rotServo.set(rotValue);
            
            
            /*
             * Allows the user to adjust the value set to the rotServo.
             */
            if(auxStick.getRawAxis(5) > 0 && rotValue <= 0.95)
            {
                rotValue = rotValue + 0.05;
            }
            else if(auxStick.getRawAxis(5) < 0 && rotValue >= 0.05)
            {
                rotValue = rotValue - 0.05;
            }
        
            lcd.println(1, 1, "winch " + winchPower + "       ");
            lcd.println(2, 1, "cam rot " + rotValue);
            lcd.println(3, 1, "input2" + armedSwitch.get() + "          ");
            lcd.updateLCD();
            
            Timer.delay(0.01);
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
                lcd.println(1, 1, " " + getHotGoal());
            }
            
            if(auxStick.getRawButton(2))
            {
                lcd.println(2, 1, " " + getVisionDistance());
            }
            
            lcd.updateLCD();
        }
    }
    
    /**
     * Moves the catapult into armed position.
     */
    private void armCatapult()
    {
        if(!armedSwitch.get())
        {
            winchMotor.set(-0.7);
        }
        else 
        {
            winchMotor.set(0);
        }
    }
    
    /**
     * Fires the catapult.
     */
    private void fireCatapult()
    {
        new Thread(new Runnable(){
                public void run()
                {
                   Timer timer = new Timer();
        timer.start();
        if(!armedSwitch.get())
        {
            while(!armedSwitch.get() && timer.get() < 0.5)
            {
                winchMotor.set(-0.4);
            }
        }
        timer.reset();
        while(armedSwitch.get() && timer.get() < 1)
        {
            winchMotor.set(-0.4);
        }
        winchMotor.set(0);
        timer.stop();
                }}).start();
        
        
//        Timer timer = new Timer();
//        timer.start();
//        if(!armedSwitch.get())
//        {
//            while(!armedSwitch.get() && timer.get() < 0.5)
//            {
//                winchMotor.set(-0.3);
//            }
//        }
//        timer.reset();
//        while(armedSwitch.get() && timer.get() < 1)
//        {
//            winchMotor.set(-0.3);
//        }
//        winchMotor.set(0);
//        timer.stop();
    }
   
    
    /**
     * Added to abstract the drive method so that CAN can be switched to PWM easier and more simply.
     * @param mX The X value of the drive vector.
     * @param mY The Y value of the drive vector.
     * @param twist The turn added to the output of the drive vector.
     * @return True if successful, false if exceptions are thrown.
     */
    private boolean tDrive(double mX, double mY, double twist)
    {
        try
        {
        return mDrive.drive(mX, mY, twist);
        }
        catch(NullPointerException ex)
        {
            ex.printStackTrace();
            return true;
        }
    }
    
    /**
     * Detects whether the hot goal is visible.
     * @return True if the hot goal is visible.  False if the hot goal is not visible or an exception has been thrown.
     */
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
                lcd.println(2, 1, "" + hulledImage.getNumberParticles());
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
        boolean successful = true;
        
        mDrive = null;
        while(aF == null || bF == null || aB == null || bB == null)
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
                
                aF.configNeutralMode(CANJaguar.NeutralMode.kBrake);
                bF.configNeutralMode(CANJaguar.NeutralMode.kBrake);
                aB.configNeutralMode(CANJaguar.NeutralMode.kBrake);
                bB.configNeutralMode(CANJaguar.NeutralMode.kBrake);

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

                //aF.setX(0);
                //bF.setX(0);
                //aB.setX(0);
                //bB.setX(0);
            }
            catch(CANTimeoutException ex)
            {
                ex.printStackTrace();
                successful = true;
            }
        }
        
        mDrive = new MecanumDrive(aF, aB, bF, bB);
        
        return successful;
    }
    
    /**
     * Detects whether one or more of the CANJaguars has lost and then regained power.
     * @return True if power to one or more of the CANJaguars has been cycled or if a timeout exception has occurred.  False otherwise.
     */
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
    
    /**
     * Filters out noise from the input of the joysticks.
     * @param joystickValue The raw input value from the joystick.
     * @return The filtered value.
     */
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
