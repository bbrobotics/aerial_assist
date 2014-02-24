package team1517.aerialassist.io;


import edu.wpi.first.wpilibj.DriverStationLCD;

/**
 *
 * @author kuroki
 */
public class DriverLCD {
    
    DriverStationLCD lcd;
    
    public DriverLCD()
    {
        /* Get instance of lcd */
        lcd = DriverStationLCD.getInstance();
    }
    
    public boolean println (int lineNum, int col, String text) {
        /* switch statement maps ints to the enums; if the linenum requested
           doesn't have a corresponding enum the method returns false */
            switch (lineNum) {
                case 1:
                    lcd.println(DriverStationLCD.Line.kUser1, col, text);
                    /* breaks from switch construct and returns true */
                    break;
                case 2:
                    lcd.println(DriverStationLCD.Line.kUser2, col, text);
                    break;
                case 3:
                    lcd.println(DriverStationLCD.Line.kUser3, col, text);
                    break;
                case 4:
                    lcd.println(DriverStationLCD.Line.kUser4, col, text);
                    break;
                case 5:
                    lcd.println(DriverStationLCD.Line.kUser5, col, text);
                    break;
                case 6:
                    lcd.println(DriverStationLCD.Line.kUser2, col, text);
                default:
                    /* if the int doesn't match any other case clauses, it
                       returns false by default */
                    return false;                    
            }
            return true;
        }
    public void push () {
        /* basically just calls the LCD update method; only advantage is that
           the main class doesn't need to import the official FIRST LCD class
           solely for this purpose */
        lcd.updateLCD();
    }
    
    
        
}