/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package team1517.aerialassist.io;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * A class for interfacing with the physical autonomous switch on the robot.
 * @author Zoraver
 * @version 1.0.0
 */
public class AutonomousSwitch {
    
    DigitalInput in1, in2, in3, in4;
    
    /**
     * Constructs the AutonomousSwitch object with four parameters.  
     * Each int represents the DigitalInput port for that position on the autonomous switch. 
     * @param channelOne    The number of the port on the digital sidecar for the first position of the autonomous switch.
     * @param channelTwo    The number of the port on the digital sidecar for the second position of the autonomous switch.
     * @param channelThree  The number of the port on the digital sidecar for the third position of the autonomous switch.
     * @param channelFour   The number of the port on the digital sidecar for the fourth position of the autonomous switch.
     */
    public AutonomousSwitch(int channelOne, int channelTwo, int channelThree, int channelFour)
    {
        in1 = new DigitalInput(channelOne);
        in2 = new DigitalInput(channelTwo);
        in3 = new DigitalInput(channelThree);
        in4 = new DigitalInput(channelFour);
    }
    
    /**
     * Gets the position of the autonomous switch and returns it as an int from one to four.  
     * Returns -1 if there is an electrical problem.
     * @return The position of the autonomous switch, with 1 for position one, 2 for position two, and so on.  Returns -1 if none of the circuits are closed.
     */
    public int getPosition()
    {
        if(in1.get())
        {
            return 1;
        }
        else if(in2.get())
        {
            return 2;
        }
        else if(in3.get())
        {
            return 3;
        }
        else if(in4.get())
        {
            return 4;
        }
        else
        {
            return -1;
        }
    }
    
    /**
     * Gets the status of position one on the autonomous switch.
     * @return The status of position one.
     */
    public boolean getPositionOne()
    {
        return in1.get();
    }
    
    /**
     * Gets the status of position two on the autonomous switch.
     * @return The status of position two.
     */
    public boolean getPositionTwo()
    {
        return in2.get();
    }
    
    /**
     * Gets the status of position three on the autonomous switch.
     * @return The status of position three.
     */
    public boolean getPositionThree()
    {
        return in3.get();
    }
    
    /**
     * Gets the status of position four on the autonomous switch.
     * @return The status of position four.
     */
    public boolean getPositionFour()
    {
        return in4.get();
    }
}
