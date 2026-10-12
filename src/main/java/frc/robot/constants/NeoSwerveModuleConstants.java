package frc.robot.constants;

import edu.wpi.first.math.util.Units;

/**
 * The NeoSwerveModuleConstants class stores constant values specific to
 * REV Robotics MAXSwerve modules utilizing NEO brushless motors.
 * This includes gearbox configurations, physical dimensions, and encoder conversion factors.
 */
public class NeoSwerveModuleConstants {

    /**
     * Gearbox options for the REV MAXSwerve Module, defining the gear ratio
     * and pinion teeth count for different speed configurations.
     */
    public enum GearboxConfig {
        LOW(5.50, 12),
        MEDIUM(5.08, 13),
        HIGH(4.71, 14);

        /** The gear reduction ratio of the gearbox. */
        public final double ratio;
        /** The number of teeth on the pinion gear. */
        public final int pinionTeeth;

        /**
         * Constructs a GearboxConfig enum member.
         * @param ratio The gear reduction ratio.
         * @param teeth The number of pinion teeth.
         */
        GearboxConfig(double ratio, int teeth) {
            this.ratio = ratio;
            this.pinionTeeth = teeth;
        }
    }

    // --- THE SETTINGS YOU CHANGE ---
    /** The currently selected gearbox configuration for the swerve modules. */
    public static final GearboxConfig kCurrentGearbox = GearboxConfig.LOW; // 12T pinion (Doyenne_2026)
    /** The diameter of the swerve module wheel in inches. */
    private static final double kWheelDiameterInches = 3.0; // (Doyenne_2026)

    // --- CALCULATIONS ---
    /** The diameter of the swerve module wheel in meters. */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches); 
    /** The circumference of the swerve module wheel in meters. */
    private static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; 

    /**
     * The theoretical maximum speed of the swerve module in meters per second.
     * This is calculated using the NEO motor's free speed, gearbox ratio, and wheel circumference.
     */
    public static final double kMaxSpeedMetersPerSecond = ((NeoMotorConstants.kFreeSpeedRpm / 60) / kCurrentGearbox.ratio) * kWheelCircumferenceMeters;

    // DRIVE ENCODER FACTORS
    /**
     * Conversion factor to turn drive motor rotations into meters.
     * Calculated as wheel circumference divided by gearbox ratio.
     */
    public static final double kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kCurrentGearbox.ratio; 
    
    /**
     * Conversion factor to turn drive motor RPM into meters per second.
     * Calculated from the position factor.
     */
    public static final double kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0; 

    // TURNING FACTORS
    /**
     * Conversion factor for the turning encoder position.
     * The absolute encoder is 1:1 with the wheel, so its factor is just one full circle (2*PI radians).
     */
    public static final double kTurningEncoderPositionFactor = 2 * Math.PI; 
    /**
     * Conversion factor for the turning encoder velocity.
     * Calculated from the position factor.
     */
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; 
}