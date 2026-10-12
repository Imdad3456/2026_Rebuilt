package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The GlobalConstants class stores general, robot-wide constant values.
 * These constants are often used across multiple subsystems, such as current limits for motors.
 */
public class GlobalConstants {
    /** High current limit, typically used for high-power mechanisms like climbers or heavy lifting systems. */
    public static final int kHighCurrentLimit = 60; 
    /** Medium current limit, often used for drive motors or heavy shooters. */
    public static final int kMediumCurrentLimit = 40; 
    /** Low current limit, suitable for lighter mechanisms such as intakes, small extenders, or turning motors. */
    public static final int kLowCurrentLimit = 20; 

    private static final String BENCHTOP_SERIAL = "030F25BC";
    
    // if the serial number is the benchtop or we are in simulation mode, IS_BENCHTOP is true
    public static final boolean IS_BENCHTOP = 
        RobotController.getSerialNumber().trim().equalsIgnoreCase(BENCHTOP_SERIAL) || 
            RobotBase.isSimulation();

}