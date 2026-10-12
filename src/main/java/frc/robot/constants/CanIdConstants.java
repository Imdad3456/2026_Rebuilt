package frc.robot.constants;

/**
 * CAN IDs for every motor controller.
 *
 * <pre>
 * Swerve turning 1, 3, 5, 7    Swerve driving 2, 4, 6, 8
 * Intake 10 (also runs the floor conveyor)
 * Extender 20-24    Climber 25-29 (not installed)
 * Feeder 30-34      Launcher 35-39 (TalonFX)
 * PDH 60
 * </pre>
 */
public final class CanIdConstants {
    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final int kFrontLeftDrivingCanId = 6;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kExtenderMotor1 = 20;
    public static final int kExtenderMotor2 = 21;

    public static final int kClimberMotor1 = 25;
    public static final int kClimberMotor2 = 26;

    public static final int kIntakeMotor = 10;

    public static final int kFeederMotor = 30;

    /** Kraken/Falcon (TalonFX), not a SPARK MAX. */
    public static final int kLauncherMotor = 35;

    private CanIdConstants() {}
}
