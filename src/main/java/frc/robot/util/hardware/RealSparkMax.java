package frc.robot.util.hardware;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

public class RealSparkMax implements MotorControllerWrapper {
    private final SparkMax m_motor;
    private final SparkClosedLoopController m_controller;
    private RelativeEncoder m_relativeEncoder;
    private AbsoluteEncoder m_absoluteEncoder;
    private boolean m_useAbsolute;
        

    public RealSparkMax(int deviceId, SparkMaxConfig config, boolean isAbsolute) {
        this.m_motor = new SparkMax(deviceId, MotorType.kBrushless);
        this.m_controller = this.m_motor.getClosedLoopController();
        
        this.m_useAbsolute = isAbsolute;
        if (isAbsolute) {
            m_absoluteEncoder = this.m_motor.getAbsoluteEncoder();
        } else {
            m_relativeEncoder = this.m_motor.getEncoder();
        }

        // You can expand this config for PID or Current Limits later
        this.m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

        this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setMaxAccel(double accel)
    {
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.maxMotion
        .maxAcceleration(accel);
        this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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
        this.m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}