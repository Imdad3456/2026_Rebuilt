package frc.robot.constants;

/** Intake settings. Speed comes from the operator's intake slider. */
public final class IntakeConstants {
    /** Intake duty cycle with the slider all the way down. Never 0, so the intake can't look dead. */
    public static final double kMinSpeed = 0.2;
    /** Intake duty cycle with the slider all the way up. */
    public static final double kMaxSpeed = 1.0;

    private IntakeConstants() {}
}
