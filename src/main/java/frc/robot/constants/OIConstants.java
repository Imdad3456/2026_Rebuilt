package frc.robot.constants;

/**
 * Controller ports, buttons, and slider axes.
 *
 * <p>To check a number: plug the controller into the Driver Station, open the USB tab, and
 * move the slider or press the button. The DS shows which axis or button changes.
 */
public final class OIConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kOperatorJoystickPort = 1;

    /** Stick movement below this is ignored. */
    public static final double kDriveDeadband = 0.1; // (Doyenne_2026)

    // ---- Driver stick ----
    /** Slider that sets drive speed. 3 is the standard joystick throttle axis. VERIFY. */
    public static final int kDriveSpeedSliderAxis = 3;
    /** Set true if pushing the drive-speed slider "up" makes the robot slower. */
    public static final boolean kDriveSpeedSliderInverted = false;
    /** Fraction of max speed with the slider all the way down. Never 0, so the robot can't look dead. */
    public static final double kMinDriveSpeedScale = 0.25;
    /** Hold to lock the wheels in an X (hard to push). (No gyro, so no heading/field-relative buttons.) */
    public static final int kXLockButton = 4;

    /** Max change in translation input per second (0 to full stick in 1/3 s). Limits current spikes. */
    public static final double kTranslationSlewRate = 3.0;
    /** Max change in rotation input per second. */
    public static final double kRotationSlewRate = 5.0;

    // ---- Operator stick ----
    /** Slider that sets shooter RPM: axis 5, as in Doyenne_2026. */
    public static final int kShooterSpeedSliderAxis = 5;
    public static final boolean kShooterSpeedSliderInverted = false;
    /** Slider that sets intake speed: the operator's other slider (the old shooter axis). VERIFY. */
    public static final int kIntakeSpeedSliderAxis = 6;
    public static final boolean kIntakeSpeedSliderInverted = false;

    // Operator buttons: the March 2026 competition layout the drivers use.
    public static final int kExtenderOutButton = 1;
    public static final int kExtenderInButton = 2;
    /** Hold: the operator stick's X axis moves the box in/out. */
    public static final int kExtenderManualButton = 3;
    public static final int kIntakeReceiveButton = 4;
    // 5 was the vision launch-mode toggle; there's no Limelight, so it's unused.
    public static final int kIntakeEjectButton = 6;
    public static final int kExtenderHomeButton = 8;
    public static final int kClearExtenderFaultsButton = 13;
    /** Stop every mechanism (box holds where it is). */
    public static final int kStopAllButton = 14;
    public static final int kLauncherIdleOnButton = 16;
    public static final int kLauncherIdleOffButton = 17;
    public static final int kLauncherButton = 18;
    /**
     * Operator stick axis for manual extender mode. The March code read the DRIVER's X axis here
     * (which also strafes the robot), so this uses the operator's own X axis instead. VERIFY.
     */
    public static final int kExtenderManualAxis = 0;

    /** Convert a slider's raw -1..1 reading to 0..1. */
    public static double sliderToPercent(double raw, boolean inverted) {
        double percent = (raw + 1.0) / 2.0;
        percent = inverted ? 1.0 - percent : percent;
        return Math.max(0.0, Math.min(1.0, percent));
    }

    private OIConstants() {}
}
