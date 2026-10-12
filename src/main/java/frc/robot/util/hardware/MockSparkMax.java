package frc.robot.util.hardware;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MockSparkMax implements MotorControllerWrapper {
    private double m_lastSpeed = 0;
    private double m_lastPos = 0;
    private double m_target = 0;
    private ControlType m_type;
    private SparkMaxConfig m_config;

    @Override
    public void set(double speed) {
        // We save the speed just for debugging, but we don't talk to CAN
        this.m_lastSpeed = speed;
    }

    @Override
    public double getPosition() {
        return this.m_lastPos; // The ghost never moves
    }

    @Override
    public double getVelocity() {
        return 0.0; // The ghost has no speed
    }

    @Override
    public void setPosition(double pos) {
        this.m_lastPos = pos;
    }

    @Override
    public double getTarget() {
        return this.m_target;
    }

    @Override
    public void setTargetValue(double value, ControlType type) {
        this.m_target = value;
        this.m_type = type;
    }

    @Override
    public void setConfiguration(SparkMaxConfig config) {
        this.m_config = config;
    }

    @Override
    public void updateHardwarePID(double p, double i, double d, double kV , double kS){
        if (this.m_config == null) {
            this.m_config = new SparkMaxConfig();
        }
        this.m_config.closedLoop.feedForward
        .kV(kV)
        .kS(kS);

        this.m_config.closedLoop.pid(p, i, d);
    }

    @Override
    public com.revrobotics.spark.SparkClosedLoopController getPIDController() {
        return null;
    }
}