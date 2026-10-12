package frc.robot.util.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;

/** The one place TalonFX (Kraken/Falcon) controllers are created, like {@link Sparks}. */
public final class Talons {
  private static final List<TalonFX> s_all = new ArrayList<>();

  private Talons() {}

  /** Create a TalonFX on the roboRIO CAN bus and apply its full config. */
  public static TalonFX create(int canId, TalonFXConfiguration config) {
    TalonFX talon = new TalonFX(canId);
    check(talon, "boot configure", talon.getConfigurator().apply(config));
    s_all.add(talon);
    return talon;
  }

  /** Every TalonFX created so far (read-only), for fault monitoring. */
  public static List<TalonFX> all() {
    return Collections.unmodifiableList(s_all);
  }

  /** Log a failed config/control call once, with the CAN ID. */
  public static void check(TalonFX talon, String what, StatusCode result) {
    if (!result.isOK()) {
      DataLogManager.log("[MOTOR] TalonFX CAN " + talon.getDeviceID() + " " + what + " FAILED: " + result);
    }
  }
}
