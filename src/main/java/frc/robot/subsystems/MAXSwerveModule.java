// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.configs.DriveConfigs;
import frc.robot.util.hardware.Sparks;

/**
 * One REV MAXSwerve module: a NEO drive motor (velocity control on its built-in encoder) and a
 * NEO steering motor (position control on the Through Bore absolute encoder).
 *
 * <p>Chassis offsets follow REV's convention: the absolute encoders read the offset when the
 * wheel points forward, so getState()/getPosition() subtract it and setDesiredState() adds it.
 * Doyenne_2026 used this and drove correctly on the current robot.
 */
public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;
  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkClosedLoopController m_drivingController;
  private final SparkClosedLoopController m_turningController;

  private final double m_chassisAngularOffset;

  /** Last angle (radians, encoder frame) and speed (m/s) sent to the controllers. */
  private double m_commandedAngle = 0;
  private double m_commandedSpeed = 0;

  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = Sparks.create(drivingCANId, DriveConfigs.MAXSwerveModule.drivingConfig);
    // Steering MUST use the turning config: absolute encoder feedback, brake mode, low current.
    m_turningSpark = Sparks.create(turningCANId, DriveConfigs.MAXSwerveModule.turningConfig);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();
    m_drivingController = m_drivingSpark.getClosedLoopController();
    m_turningController = m_turningSpark.getClosedLoopController();

    m_chassisAngularOffset = chassisAngularOffset;
    m_drivingEncoder.setPosition(0);
  }

  /** Current velocity and angle, relative to the chassis. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /** Distance driven and angle, relative to the chassis. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /** Steer and drive toward the requested state, never turning more than 90 degrees. */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Chassis frame -> this module's encoder frame.
    SwerveModuleState state = new SwerveModuleState(desiredState.speedMetersPerSecond,
        desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));
    Rotation2d currentRotation = new Rotation2d(m_turningEncoder.getPosition());

    state.optimize(currentRotation);
    // Don't drive hard until the wheel is pointing the right way.
    state.cosineScale(currentRotation);

    m_commandedAngle = state.angle.getRadians();
    m_commandedSpeed = state.speedMetersPerSecond;
    m_drivingController.setSetpoint(m_commandedSpeed, ControlType.kVelocity);
    m_turningController.setSetpoint(m_commandedAngle, ControlType.kPosition);
  }

  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void setDriveGains(double p, double i, double d, double kV, double kS) {
    Sparks.setGains(m_drivingSpark, p, i, d, kV, kS);
  }

  public void setTurnGains(double p, double i, double d, double kV, double kS) {
    Sparks.setGains(m_turningSpark, p, i, d, kV, kS);
  }

  /** Last commanded steering angle in radians, in the absolute encoder's frame. */
  public double getCommandedAngle() {
    return m_commandedAngle;
  }

  /** Measured steering angle in radians, in the absolute encoder's frame. */
  public double getMeasuredAngle() {
    return m_turningEncoder.getPosition();
  }

  public double getCommandedSpeed() {
    return m_commandedSpeed;
  }

  public double getMeasuredSpeed() {
    return m_drivingEncoder.getVelocity();
  }
}
