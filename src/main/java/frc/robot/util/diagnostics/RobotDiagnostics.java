package frc.robot.util.diagnostics;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.OIConstants;
import frc.robot.util.hardware.Sparks;
import frc.robot.util.hardware.Talons;

/**
 * Competition-safe diagnostics, written only to the on-robot .wpilog (no NetworkTables
 * traffic, no extra CAN requests). Built to tell apart the causes of "auto worked,
 * then the whole robot stopped responding in teleop":
 *
 * <ul>
 *   <li>Driver Station / FMS comms loss: DS attached, comms-disable count, sys active</li>
 *   <li>Joystick problems: per-port connected/name/type (axes + buttons come from
 *       DriverStation.startDataLog)</li>
 *   <li>Code crash or stall: program start + FPGA uptime, loop period, periodic time,
 *       exceptions</li>
 *   <li>Brownout: battery voltage, brownout flag</li>
 *   <li>CAN: bus utilization and error counters, per-SPARK faults, warnings and resets</li>
 * </ul>
 *
 * <p>Every value uses {@code update()}, which only writes when the value changes, and
 * the slower signals are sampled every few loops. Console output only happens for rare
 * state transitions.
 *
 * <p>Open the log in AdvantageScope (or WPILib's DataLogTool). Everything is under
 * {@code /Diag/}, and {@code /Diag/Events} is a readable timeline.
 */
public final class RobotDiagnostics {
  private static final int[] kJoystickPorts = {0, 1};
  /** A loop slower than this counts as an overrun (nominal is 0.020 s). */
  private static final double kOverrunSeconds = 0.025;
  private static final int kPowerEveryNLoops = 5; // 10 Hz
  private static final int kSlowEveryNLoops = 25; // 2 Hz
  /** FPGA uptime below this at program start means the roboRIO itself rebooted. */
  private static final double kFreshBootSeconds = 90.0;

  private static RobotDiagnostics s_instance;

  private final StringLogEntry m_events;

  // Robot / DS state
  private final BooleanLogEntry m_enabled;
  private final BooleanLogEntry m_autonomous;
  private final BooleanLogEntry m_teleop;
  private final BooleanLogEntry m_fmsAttached;
  private final BooleanLogEntry m_dsAttached;
  private final BooleanLogEntry m_sysActive;
  private final BooleanLogEntry m_eStopped;
  private final IntegerLogEntry m_commsDisableCount;
  private final DoubleLogEntry m_matchTime;

  // Power
  private final DoubleLogEntry m_batteryVoltage;
  private final BooleanLogEntry m_brownedOut;

  // CAN bus
  private final DoubleLogEntry m_canUtilization;
  private final IntegerLogEntry m_canBusOff;
  private final IntegerLogEntry m_canTxFull;
  private final IntegerLogEntry m_canRxErrors;
  private final IntegerLogEntry m_canTxErrors;

  // Joysticks
  private final BooleanLogEntry[] m_joyConnected = new BooleanLogEntry[kJoystickPorts.length];
  private final StringLogEntry[] m_joyDescription = new StringLogEntry[kJoystickPorts.length];

  // Loop timing
  private final DoubleLogEntry m_loopPeriod;
  private final DoubleLogEntry m_periodicTime;
  private final IntegerLogEntry m_overrunCount;

  // Drive input path
  private final BooleanLogEntry m_driveCommandScheduled;
  private final DoubleLogEntry m_driveX;
  private final DoubleLogEntry m_driveY;
  private final DoubleLogEntry m_driveRot;
  private final DoubleLogEntry m_driveSpeedScale;

  // Motors (one SPARK checked per loop, round robin)
  private final List<SparkMax> m_motors;
  private final IntegerLogEntry[] m_motorFaults;
  private final IntegerLogEntry[] m_motorWarnings;
  private final StringLogEntry[] m_motorLastError;
  private final BooleanLogEntry[] m_motorFwdLimit;
  private final BooleanLogEntry[] m_motorRevLimit;
  private final boolean[] m_prevLimitPressed;
  private final int[] m_prevFaultBits;
  private final int[] m_prevWarningBits;
  private int m_nextMotor = 0;
  private final boolean[] m_sparkOk;

  // TalonFX (launcher)
  private final List<TalonFX> m_talons;
  private final BooleanLogEntry[] m_talonConnected;
  private final IntegerLogEntry[] m_talonFaults;
  private final BooleanLogEntry[] m_talonBootDuringEnable;
  private final boolean[] m_talonOk;
  private final boolean[] m_prevTalonConnected;
  private final int[] m_prevTalonFaultBits;
  private String m_prevHealthProblems = null;
  private String m_prevControllerStatus = null;

  // Edge-detection state
  private boolean m_prevDsAttached = false;
  private boolean m_prevBrownedOut = false;
  private boolean m_prevSysActive = false;
  private int m_prevCommsDisableCount;
  private int m_prevBusOffCount;
  private final boolean[] m_prevJoyConnected = new boolean[kJoystickPorts.length];
  private String m_prevMode = "";

  private Command m_driveCommand;
  private long m_loopCount = 0;
  private long m_overruns = 0;
  private double m_lastLoopStart = Double.NaN;
  private double m_lastSlowLoopEvent = Double.NEGATIVE_INFINITY;

  private RobotDiagnostics(String buildDescription) {
    DataLog log = DataLogManager.getLog();
    m_events = new StringLogEntry(log, "/Diag/Events");

    m_enabled = new BooleanLogEntry(log, "/Diag/State/Enabled");
    m_autonomous = new BooleanLogEntry(log, "/Diag/State/Autonomous");
    m_teleop = new BooleanLogEntry(log, "/Diag/State/Teleop");
    m_fmsAttached = new BooleanLogEntry(log, "/Diag/State/FMSAttached");
    m_dsAttached = new BooleanLogEntry(log, "/Diag/State/DSAttached");
    m_sysActive = new BooleanLogEntry(log, "/Diag/State/OutputsActive");
    m_eStopped = new BooleanLogEntry(log, "/Diag/State/EStopped");
    m_commsDisableCount = new IntegerLogEntry(log, "/Diag/State/CommsDisableCount");
    m_matchTime = new DoubleLogEntry(log, "/Diag/State/MatchTime");

    m_batteryVoltage = new DoubleLogEntry(log, "/Diag/Power/BatteryVoltage");
    m_brownedOut = new BooleanLogEntry(log, "/Diag/Power/BrownedOut");

    m_canUtilization = new DoubleLogEntry(log, "/Diag/CAN/Utilization");
    m_canBusOff = new IntegerLogEntry(log, "/Diag/CAN/BusOffCount");
    m_canTxFull = new IntegerLogEntry(log, "/Diag/CAN/TxFullCount");
    m_canRxErrors = new IntegerLogEntry(log, "/Diag/CAN/ReceiveErrorCount");
    m_canTxErrors = new IntegerLogEntry(log, "/Diag/CAN/TransmitErrorCount");

    for (int i = 0; i < kJoystickPorts.length; i++) {
      int port = kJoystickPorts[i];
      m_joyConnected[i] = new BooleanLogEntry(log, "/Diag/Joystick" + port + "/Connected");
      m_joyDescription[i] = new StringLogEntry(log, "/Diag/Joystick" + port + "/Description");
    }

    m_loopPeriod = new DoubleLogEntry(log, "/Diag/Loop/PeriodSeconds");
    m_periodicTime = new DoubleLogEntry(log, "/Diag/Loop/RobotPeriodicSeconds");
    m_overrunCount = new IntegerLogEntry(log, "/Diag/Loop/OverrunCount");

    m_driveCommandScheduled = new BooleanLogEntry(log, "/Diag/Drive/DriveCommandScheduled");
    m_driveX = new DoubleLogEntry(log, "/Diag/Drive/InputX");
    m_driveY = new DoubleLogEntry(log, "/Diag/Drive/InputY");
    m_driveRot = new DoubleLogEntry(log, "/Diag/Drive/InputRot");
    m_driveSpeedScale = new DoubleLogEntry(log, "/Diag/Drive/SpeedScale");

    // Snapshot: a motor created later must not outgrow the per-motor arrays below.
    m_motors = List.copyOf(Sparks.all());
    int n = m_motors.size();
    m_motorFaults = new IntegerLogEntry[n];
    m_motorWarnings = new IntegerLogEntry[n];
    m_motorLastError = new StringLogEntry[n];
    m_motorFwdLimit = new BooleanLogEntry[n];
    m_motorRevLimit = new BooleanLogEntry[n];
    m_prevLimitPressed = new boolean[n];
    m_prevFaultBits = new int[n];
    m_prevWarningBits = new int[n];
    for (int i = 0; i < n; i++) {
      String prefix = String.format("/Diag/Motors/CAN%02d/", m_motors.get(i).getDeviceId());
      m_motorFaults[i] = new IntegerLogEntry(log, prefix + "FaultBits");
      m_motorWarnings[i] = new IntegerLogEntry(log, prefix + "WarningBits");
      m_motorLastError[i] = new StringLogEntry(log, prefix + "LastError");
      m_motorFwdLimit[i] = new BooleanLogEntry(log, prefix + "ForwardLimitPressed");
      m_motorRevLimit[i] = new BooleanLogEntry(log, prefix + "ReverseLimitPressed");
    }

    m_sparkOk = new boolean[n];
    java.util.Arrays.fill(m_sparkOk, true);

    m_talons = List.copyOf(Talons.all());
    int t = m_talons.size();
    m_talonConnected = new BooleanLogEntry[t];
    m_talonFaults = new IntegerLogEntry[t];
    m_talonBootDuringEnable = new BooleanLogEntry[t];
    m_talonOk = new boolean[t];
    java.util.Arrays.fill(m_talonOk, true);
    m_prevTalonConnected = new boolean[t];
    java.util.Arrays.fill(m_prevTalonConnected, true);
    m_prevTalonFaultBits = new int[t];
    for (int i = 0; i < t; i++) {
      String prefix = String.format("/Diag/Motors/TalonFX%02d/", m_talons.get(i).getDeviceID());
      m_talonConnected[i] = new BooleanLogEntry(log, prefix + "Connected");
      m_talonFaults[i] = new IntegerLogEntry(log, prefix + "FaultBits");
      m_talonBootDuringEnable[i] = new BooleanLogEntry(log, prefix + "RebootedWhileEnabled");
    }

    m_prevCommsDisableCount = RobotController.getCommsDisableCount();
    m_prevBusOffCount = RobotController.getCANStatus().busOffCount;

    double uptime = RobotController.getFPGATime() / 1e6;
    event(String.format(
        "ROBOT PROGRAM START %s | roboRIO uptime %.1f s (%s) | brownout threshold %.2f V | %d SPARKs, %d TalonFX",
        buildDescription, uptime,
        uptime < kFreshBootSeconds ? "roboRIO BOOTED - power was lost or rio rebooted"
            : "roboRIO was already running - robot CODE restarted (crash or redeploy)",
        RobotController.getBrownoutVoltage(), n, t),
        true);

    registerSchedulerCallbacks();
  }

  /** Create the diagnostics. Call once, after all motors are constructed. */
  public static RobotDiagnostics start(String buildDescription) {
    if (s_instance == null) {
      s_instance = new RobotDiagnostics(buildDescription);
    }
    return s_instance;
  }

  /** Latest motor health summary ("All motors OK" or which ones have problems). */
  public static String healthReport() {
    return s_instance == null || s_instance.m_prevHealthProblems == null ? "All motors OK" : s_instance.m_prevHealthProblems;
  }

  public static boolean allMotorsOk() {
    return healthReport().equals("All motors OK");
  }

  /** Latest controller check ("Both controllers OK" or what's wrong). */
  public static String controllerStatus() {
    return s_instance == null || s_instance.m_prevControllerStatus == null ? "Both controllers OK" : s_instance.m_prevControllerStatus;
  }

  public static boolean controllersOk() {
    return controllerStatus().equals("Both controllers OK");
  }

  /** Record the joystick values DriveCommand actually sends to the drivetrain. */
  public static void recordDriveInput(double x, double y, double rot, double speedScale) {
    if (s_instance != null) {
      s_instance.m_driveSpeedScale.update(speedScale);
      s_instance.m_driveX.update(x);
      s_instance.m_driveY.update(y);
      s_instance.m_driveRot.update(rot);
    }
  }

  /** Log a timestamped event. Use for rare transitions only. */
  public static void logEvent(String message) {
    if (s_instance != null) {
      s_instance.event(message, false);
    }
  }

  /** The drivetrain's default command, so we can log whether it is actually running. */
  public void setDriveCommand(Command driveCommand) {
    m_driveCommand = driveCommand;
  }

  /** Call at the very start of robotPeriodic(). */
  public void loopStart(double now) {
    if (!Double.isNaN(m_lastLoopStart)) {
      double period = now - m_lastLoopStart;
      m_loopPeriod.update(period);
      if (period > kOverrunSeconds) {
        m_overruns++;
        m_overrunCount.update(m_overruns);
      }
    }
    m_lastLoopStart = now;
  }

  /** Call at the end of robotPeriodic(), after the scheduler has run. */
  public void periodic(double loopStartTime) {
    m_loopCount++;

    sampleRobotState();
    if (m_loopCount % kPowerEveryNLoops == 0) {
      samplePowerAndJoysticks();
    }
    if (m_loopCount % kSlowEveryNLoops == 0) {
      sampleCan();
      sampleTalons();
      publishHealth();
      publishControllerCheck();
    }
    sampleNextMotor();

    if (m_driveCommand != null) {
      m_driveCommandScheduled.update(m_driveCommand.isScheduled());
    }

    double periodicTime = Timer.getFPGATimestamp() - loopStartTime;
    m_periodicTime.update(periodicTime);
    // Rate-limited to once a second so a stuck loop can't flood the log.
    if (periodicTime > kOverrunSeconds && loopStartTime - m_lastSlowLoopEvent > 1.0) {
      event(String.format("SLOW LOOP: robotPeriodic took %.1f ms", periodicTime * 1000), false);
      m_lastSlowLoopEvent = loopStartTime;
    }
  }

  /** Log an exception that is about to crash the robot program. */
  public static void recordException(Throwable t) {
    if (s_instance != null) {
      StackTraceElement[] st = t.getStackTrace();
      String where = st.length > 0 ? st[0].toString() : "unknown";
      s_instance.event("EXCEPTION (robot code will restart): " + t + " at " + where, true);
    }
  }

  private void sampleRobotState() {
    boolean enabled = DriverStation.isEnabled();
    boolean auto = DriverStation.isAutonomous();
    boolean teleop = DriverStation.isTeleop();
    boolean ds = DriverStation.isDSAttached();
    boolean sysActive = RobotController.isSysActive();
    int commsDisables = RobotController.getCommsDisableCount();

    m_enabled.update(enabled);
    m_autonomous.update(auto);
    m_teleop.update(teleop);
    m_fmsAttached.update(DriverStation.isFMSAttached());
    m_dsAttached.update(ds);
    m_sysActive.update(sysActive);
    m_eStopped.update(DriverStation.isEStopped());
    m_commsDisableCount.update(commsDisables);

    String mode = (!enabled ? "DISABLED"
        : auto ? "ENABLED AUTO" : DriverStation.isTest() ? "ENABLED TEST" : "ENABLED TELEOP")
        + (DriverStation.isFMSAttached() ? " [FMS]" : "");
    if (!mode.equals(m_prevMode)) {
      event("MODE -> " + mode, false);
      m_prevMode = mode;
    }

    if (ds != m_prevDsAttached) {
      event(ds ? "DS CONNECTED" : "DS DISCONNECTED - lost Driver Station comms", !ds);
      m_prevDsAttached = ds;
    }
    if (commsDisables != m_prevCommsDisableCount) {
      event("COMMS DISABLE: outputs disabled by lost DS packets (count " + commsDisables + ")", true);
      m_prevCommsDisableCount = commsDisables;
    }
    // Outputs dropping while we still think we're enabled = brownout/comms/e-stop.
    if (m_prevSysActive && !sysActive && enabled) {
      event("OUTPUTS DISABLED WHILE ENABLED (brownout, comms loss, or e-stop)", true);
    }
    m_prevSysActive = sysActive;
  }

  private void samplePowerAndJoysticks() {
    m_batteryVoltage.update(RobotController.getBatteryVoltage());
    m_matchTime.update(DriverStation.getMatchTime());

    boolean brownedOut = RobotController.isBrownedOut();
    m_brownedOut.update(brownedOut);
    if (brownedOut != m_prevBrownedOut) {
      event(brownedOut
          ? String.format("BROWNOUT START (battery %.2f V)", RobotController.getBatteryVoltage())
          : "BROWNOUT END", brownedOut);
      m_prevBrownedOut = brownedOut;
    }

    for (int i = 0; i < kJoystickPorts.length; i++) {
      int port = kJoystickPorts[i];
      boolean connected = DriverStation.isJoystickConnected(port);
      m_joyConnected[i].update(connected);
      if (connected != m_prevJoyConnected[i] || (m_loopCount % kSlowEveryNLoops == 0)) {
        String description = connected
            ? String.format("%s (type %d, %d axes, %d buttons)", DriverStation.getJoystickName(port),
                DriverStation.getJoystickType(port), DriverStation.getStickAxisCount(port),
                DriverStation.getStickButtonCount(port))
            : "not connected";
        m_joyDescription[i].update(description);
        if (connected != m_prevJoyConnected[i]) {
          event("JOYSTICK PORT " + port + (connected ? " CONNECTED: " + description : " DISCONNECTED"),
              !connected);
          m_prevJoyConnected[i] = connected;
        }
      }
    }
  }

  private void sampleCan() {
    CANStatus status = RobotController.getCANStatus();
    m_canUtilization.update(status.percentBusUtilization);
    m_canBusOff.update(status.busOffCount);
    m_canTxFull.update(status.txFullCount);
    m_canRxErrors.update(status.receiveErrorCount);
    m_canTxErrors.update(status.transmitErrorCount);
    if (status.busOffCount != m_prevBusOffCount) {
      event("CAN BUS-OFF (count " + status.busOffCount + ") - wiring/termination fault likely", true);
      m_prevBusOffCount = status.busOffCount;
    }
  }

  /** Check one SPARK per loop. These read cached status frames: no CAN requests. */
  private void sampleNextMotor() {
    if (m_motors.isEmpty()) {
      return;
    }
    int i = m_nextMotor;
    m_nextMotor = (m_nextMotor + 1) % m_motors.size();

    SparkMax spark = m_motors.get(i);
    Faults faults = spark.getFaults();
    Warnings warnings = spark.getWarnings();
    REVLibError lastError = spark.getLastError();

    // No mechanism has limit switches, so a "pressed" reading is noise on the data port.
    boolean fwdLimit = spark.getForwardLimitSwitch().isPressed();
    boolean revLimit = spark.getReverseLimitSwitch().isPressed();
    m_motorFwdLimit[i].update(fwdLimit);
    m_motorRevLimit[i].update(revLimit);
    if ((fwdLimit || revLimit) && !m_prevLimitPressed[i]) {
      event("SPARK CAN " + spark.getDeviceId() + ": limit input reads PRESSED (" + (fwdLimit ? "fwd " : "")
          + (revLimit ? "rev" : "") + ") - no limit switches are installed; check the data-port wiring", false);
    }
    m_prevLimitPressed[i] = fwdLimit || revLimit;

    m_motorFaults[i].update(faults.rawBits);
    m_motorWarnings[i].update(warnings.rawBits);
    m_motorLastError[i].update(lastError.toString());

    int newFaults = faults.rawBits & ~m_prevFaultBits[i];
    int newWarnings = warnings.rawBits & ~m_prevWarningBits[i];
    if (newFaults != 0 || newWarnings != 0) {
      event("SPARK CAN " + spark.getDeviceId() + ":" + describe(faults, warnings), true);
    }
    m_prevFaultBits[i] = faults.rawBits;
    m_prevWarningBits[i] = warnings.rawBits;
    m_sparkOk[i] = !faults.can
        && lastError != REVLibError.kCANDisconnected && lastError != REVLibError.kTimeout;
  }

  /** TalonFX health from signals the launcher already streams (no extra CAN requests). */
  private void sampleTalons() {
    for (int i = 0; i < m_talons.size(); i++) {
      TalonFX talon = m_talons.get(i);
      boolean connected = talon.isConnected();
      StatusSignal<Integer> faults = talon.getFaultField().refresh();
      StatusSignal<Boolean> rebooted = talon.getStickyFault_BootDuringEnable().refresh();
      int faultBits = (int) faults.getValueAsDouble();
      boolean bootDuringEnable = rebooted.getValueAsDouble() != 0;

      m_talonConnected[i].update(connected);
      m_talonFaults[i].update(faultBits);
      m_talonBootDuringEnable[i].update(bootDuringEnable);

      if (connected != m_prevTalonConnected[i]) {
        event("TalonFX CAN " + talon.getDeviceID() + (connected ? " RECONNECTED" : " DISCONNECTED"), !connected);
        m_prevTalonConnected[i] = connected;
      }
      int newFaults = faultBits & ~m_prevTalonFaultBits[i];
      if (newFaults != 0) {
        event(String.format("TalonFX CAN %d: new fault bits 0x%X (see Phoenix Tuner for names)",
            talon.getDeviceID(), newFaults), true);
      }
      m_prevTalonFaultBits[i] = faultBits;
      m_talonOk[i] = connected && faultBits == 0;
    }
  }

  /**
   * Pre-match controller check for the drive team. Each port must have a controller with enough
   * axes and buttons for what the code reads from it, which catches a missing stick and (usually)
   * swapped USB order. Display only.
   */
  private void publishControllerCheck() {
    String driver = checkController("Driver", OIConstants.kDriverJoystickPort,
        Math.max(OIConstants.kDriveSpeedSliderAxis, 2) + 1, OIConstants.kXLockButton);
    String operator = checkController("Operator", OIConstants.kOperatorJoystickPort,
        Math.max(OIConstants.kExtenderManualAxis,
            Math.max(OIConstants.kShooterSpeedSliderAxis, OIConstants.kIntakeSpeedSliderAxis)) + 1,
        OIConstants.kLauncherButton);
    boolean ok = driver == null && operator == null;
    String status = ok ? "Both controllers OK"
        : ((driver != null ? driver + " " : "") + (operator != null ? operator : "")).trim();
    if (!status.equals(m_prevControllerStatus)) {
      SmartDashboard.putBoolean("Controllers/OK", ok);
      SmartDashboard.putString("Controllers/Status", status);
      if (m_prevControllerStatus != null || !ok) {
        event("CONTROLLER CHECK: " + status, !ok && DriverStation.isDSAttached());
      }
      m_prevControllerStatus = status;
    }
  }

  /** Null if fine, otherwise what's wrong. */
  private static String checkController(String role, int port, int axesNeeded, int buttonsNeeded) {
    if (!DriverStation.isJoystickConnected(port)) {
      return role + " (port " + port + ") not connected.";
    }
    int axes = DriverStation.getStickAxisCount(port);
    int buttons = DriverStation.getStickButtonCount(port);
    if (axes < axesNeeded || buttons < buttonsNeeded) {
      return String.format("%s (port %d) is '%s' with %d axes/%d buttons, needs %d/%d - wrong stick or swapped?",
          role, port, DriverStation.getJoystickName(port), axes, buttons, axesNeeded, buttonsNeeded);
    }
    return null;
  }

  /**
   * Dashboard health light (idea from Doyenne_2026). Only for the drive team to see; it never
   * disables controls, because a false alarm there would make buttons silently stop working.
   */
  private void publishHealth() {
    StringBuilder problems = new StringBuilder();
    for (int i = 0; i < m_motors.size(); i++) {
      if (!m_sparkOk[i]) {
        problems.append("SPARK ").append(m_motors.get(i).getDeviceId()).append(' ');
      }
    }
    for (int i = 0; i < m_talons.size(); i++) {
      if (!m_talonOk[i]) {
        problems.append("TalonFX ").append(m_talons.get(i).getDeviceID()).append(' ');
      }
    }
    String report = problems.length() == 0 ? "All motors OK" : "PROBLEM: " + problems.toString().trim();
    if (!report.equals(m_prevHealthProblems)) {
      SmartDashboard.putBoolean("Health/All Motors OK", problems.length() == 0);
      SmartDashboard.putString("Health/Report", report);
      m_prevHealthProblems = report;
    }
  }

  private static String describe(Faults f, Warnings w) {
    StringBuilder sb = new StringBuilder();
    if (w.hasReset) sb.append(" RESET(rebooted - power dip/brownout)");
    if (w.brownout) sb.append(" brownout");
    if (w.overcurrent) sb.append(" overcurrent");
    if (w.stall) sb.append(" stall");
    if (w.sensor) sb.append(" sensor-warning");
    if (f.can) sb.append(" CAN-FAULT");
    if (f.sensor) sb.append(" SENSOR-FAULT");
    if (f.temperature) sb.append(" OVER-TEMP");
    if (f.gateDriver) sb.append(" GATE-DRIVER");
    if (f.motorType) sb.append(" MOTOR-TYPE");
    if (f.firmware) sb.append(" FIRMWARE");
    if (f.escEeprom || w.escEeprom || w.extEeprom) sb.append(" EEPROM");
    if (f.other || w.other) sb.append(" other");
    return sb.toString();
  }

  private void registerSchedulerCallbacks() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(cmd -> event("CMD START " + cmd.getName(), false));
    scheduler.onCommandFinish(cmd -> event("CMD END " + cmd.getName(), false));
    scheduler.onCommandInterrupt((Command cmd, Optional<Command> by) -> event(
        "CMD INTERRUPTED " + cmd.getName() + by.map(c -> " by " + c.getName()).orElse(""), false));
  }

  /**
   * Append to the event log. {@code alsoWarnDs} also sends it to the Driver Station
   * console. Only used for rare, important transitions so it can't spam.
   */
  private void event(String message, boolean alsoWarnDs) {
    m_events.append(message);
    if (alsoWarnDs) {
      DriverStation.reportWarning("[DIAG] " + message, false);
    }
  }
}
