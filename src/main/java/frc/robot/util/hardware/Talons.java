package frc.robot.util.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
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

  /**
   * The faults worth naming in the log. The raw fault field's bit order isn't documented, so the
   * Q2/Q6/Q8 logs only said "0x40 / 0x100 / 0x200"; these say which fault it actually was.
   */
  private static List<NamedFault> faults(TalonFX t) {
    return List.of(
        new NamedFault("hardware", t.getFault_Hardware()),
        new NamedFault("processor-temp", t.getFault_ProcTemp()),
        new NamedFault("motor-temp", t.getFault_DeviceTemp()),
        new NamedFault("undervoltage", t.getFault_Undervoltage()),
        new NamedFault("rebooted-while-enabled", t.getFault_BootDuringEnable()),
        new NamedFault("bridge-brownout", t.getFault_BridgeBrownout()),
        new NamedFault("supply-voltage-too-high", t.getFault_OverSupplyV()),
        new NamedFault("supply-voltage-unstable", t.getFault_UnstableSupplyV()),
        new NamedFault("stator-current-limit", t.getFault_StatorCurrLimit()),
        new NamedFault("supply-current-limit", t.getFault_SupplyCurrLimit()),
        new NamedFault("static-brake-disabled", t.getFault_StaticBrakeDisabled()));
  }

  private record NamedFault(String name, StatusSignal<Boolean> signal) {}

  /** Keep the named fault signals streaming (call before optimizeBusUtilization()). */
  public static void streamFaultSignals(TalonFX talon, double hz) {
    BaseStatusSignal.setUpdateFrequencyForAll(hz,
        faults(talon).stream().map(NamedFault::signal).toArray(BaseStatusSignal[]::new));
  }

  /** Names of the faults active right now, e.g. "supply-current-limit bridge-brownout". */
  public static String activeFaultNames(TalonFX talon) {
    StringBuilder sb = new StringBuilder();
    for (NamedFault f : faults(talon)) {
      if (Boolean.TRUE.equals(f.signal().refresh().getValue())) {
        sb.append(sb.length() > 0 ? " " : "").append(f.name());
      }
    }
    return sb.length() > 0 ? sb.toString() : "unnamed";
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
