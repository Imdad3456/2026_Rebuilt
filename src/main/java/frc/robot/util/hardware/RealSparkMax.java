package frc.robot.util.hardware;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DataLogManager;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

public class RealSparkMax implements MotorControllerWrapper {
    /** Every real SPARK MAX created, so diagnostics can watch their faults. */
    private static final List<RealSparkMax> s_instances = new ArrayList<>();

    private final int m_deviceId;
    private final SparkMax m_motor;
    private final SparkClosedLoopController m_controller;
    private RelativeEncoder m_relativeEncoder;
    private AbsoluteEncoder m_absoluteEncoder;
    private boolean m_useAbsolute;
        

    public RealSparkMax(int deviceId, SparkMaxConfig config, boolean isAbsolute) {
        this.m_deviceId = deviceId;
        this.m_motor = new SparkMax(deviceId, MotorType.kBrushless);
        this.m_controller = this.m_motor.getClosedLoopController();
        
        this.m_useAbsolute = isAbsolute;
        if (isAbsolute) {
            m_absoluteEncoder = this.m_motor.getAbsoluteEncoder();
        } else {
            m_relativeEncoder = this.m_motor.getEncoder();
        }

        // Persisted so a SPARK that browns out and reboots mid-match keeps its config.
        checkResult("boot configure",
            this.m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        s_instances.add(this);
    }

    /** All real SPARK MAX controllers created so far (read-only). */
    public static List<RealSparkMax> getInstances() {
        return Collections.unmodifiableList(s_instances);
    }

    public int getDeviceId() {
        return m_deviceId;
    }

    /**
     * Report a failed configure() once. A missing/duplicate CAN ID or a timeout here
     * means the motor is running on whatever config is in its flash, not ours.
     */
    private void checkResult(String what, REVLibError result) {
        if (result != REVLibError.kOk) {
            DataLogManager.log("[MOTOR] CAN " + m_deviceId + " " + what + " FAILED: " + result);
        }
    }

    @Override
    public SparkClosedLoopController getPIDController() {
        return this.m_controller;
    }

    @Override
    public void set(double speed) {
        this.m_motor.set(speed);
    }

    @Override
    public void setPosition(double position) {
        // Absolute encoders cannot be re-zeroed; only the relative encoder can.
        if (m_useAbsolute) {
            return;
        }
        this.m_relativeEncoder.setPosition(position);
    }

    @Override
    public void setTargetValue(double value, ControlType controlType) {
        // This replaces the "m_ClosedLoopController.setReference" in your launcher
        this.m_controller.setSetpoint(value, controlType);
    }

    @Override
    public double getPosition() {
        return m_useAbsolute ? m_absoluteEncoder.getPosition() : m_relativeEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return m_useAbsolute ? m_absoluteEncoder.getVelocity() : m_relativeEncoder.getVelocity();
    }

    @Override
    public void setConfiguration(SparkMaxConfig config){
        checkResult("setConfiguration",
            this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    };

    @Override
    public void setOutputVoltage(double voltage){
        this.m_motor.setVoltage(voltage);
    }

    @Override
    public double getOutputCurrent() { 
        return this.m_motor.getOutputCurrent(); 
    }

    @Override
    public void updateHardwarePID(double p, double i, double d, double kV , double kS){
        SparkMaxConfig config = new SparkMaxConfig();

        // Do NOT set feedbackSensor here: that would silently switch the swerve
        // turning motors off their absolute encoder whenever PID is retuned.
        config.closedLoop.pid(p, i, d);
        
        config.closedLoop.feedForward
        .kV(kV)
        .kS(kS);

        checkResult("PID update",
            this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public void setMaxAccel(double accel)
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.maxMotion
        .maxAcceleration(accel);
        checkResult("maxAccel", this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    @Override
    public SparkMax getSparkMax() {
        return this.m_motor;
    }

    @Override
    public void follow(MotorControllerWrapper leader, boolean invert) {
        SparkMaxConfig config = new SparkMaxConfig();
        // Follow the LEADER's SparkMax (this previously passed m_motor, so the
        // follower was following itself and never moved with the leader).
        SparkMax leaderMotor = leader.getSparkMax();
        if (leaderMotor == null) {
            return;
        }
        config.follow(leaderMotor, invert);
        checkResult("follow", this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    }
}