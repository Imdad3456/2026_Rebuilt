package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class TunableNumber {
    private final DoubleEntry entry;
    private double lastValue;

    public TunableNumber(String path, double defaultValue) {
        this.entry = NetworkTableInstance.getDefault()
            .getDoubleTopic("Tuning/" + path)
            .getEntry(defaultValue, PubSubOption.periodic(0.02));
        this.lastValue = defaultValue;
    }

    public double get() { return entry.get(); }

    /**
     * True once each time the dashboard value changes. Subsystems use this to push new
     * PID/feedforward gains to the motor controllers while disabled.
     *
     * <p>Always false when tuning mode is off or when connected to the FMS. Rewriting
     * motor configs is a blocking CAN operation, and on the field the robot is disabled
     * for a few seconds between auto and teleop. A dashboard change (or a dashboard
     * re-sending its values after reconnecting) must not rewrite every motor's config
     * in that window. The change is still picked up after the match.
     */
    public boolean hasChanged() {
        if (!TuningManager.kTuningMode || DriverStation.isFMSAttached()) {
            return false;
        }
        double currentValue = get();
        if (currentValue != lastValue) {
            lastValue = currentValue;
            return true;
        }
        return false;
    }
}
