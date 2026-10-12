// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.configs.DriveConfigs;
import frc.robot.util.hardware.HardwareFactory;
import frc.robot.util.hardware.MotorControllerWrapper;

/**
 * The MAXSwerveModule class represents a single swerve drive module,
 * encapsulating its driving and turning motors, their associated encoders,
 * and PID controllers for precise control.
 * This configuration is specific to the REV MAXSwerve Module built with NEOs,
 * SPARKS MAX motor controllers, and a Through Bore Encoder for absolute
 * positioning.
 */
public class MAXSwerveModule {
  /** Motor controller for the drive wheel. */
  private final MotorControllerWrapper m_drivingMotor;
  /** Motor controller for the turning (steer) motor. */
  private final MotorControllerWrapper m_turningMotor;

  /** Angular offset of this module relative to the chassis. */
  private double m_chassisAngularOffset = 0;

  /**
   * Constructs a MAXSwerveModule and configures its driving and turning motors,
   * encoders, and PID controllers.
   * This configuration is tailored for the REV MAXSwerve Module using NEO motors,
   * SPARK MAX motor controllers, and a Through Bore Encoder.
   *
   * @param drivingCANId         The CAN ID of the SparkMax controller for the
   *                             driving motor.
   * @param turningCANId         The CAN ID of the SparkMax controller for the
   *                             turning motor.
   * @param chassisAngularOffset The angular offset of this module relative to the
   *                             chassis, in radians.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingMotor = HardwareFactory.createSparkMax(drivingCANId, DriveConfigs.MAXSwerveModule.drivingConfig);
    // Turning motors MUST use the turning config (absolute encoder feedback, brake mode,
    // low current limit). Using the driving config here gave them a 60A limit and coast
    // mode, and closed the loop on the wrong encoder.
    m_turningMotor = HardwareFactory.createSparkMax(turningCANId, DriveConfigs.MAXSwerveModule.turningConfig, true);

    this.m_chassisAngularOffset = chassisAngularOffset;
    // Set the driving encoder's position to zero initially.
    m_drivingMotor.setPosition(0);
  }

  /**
   * Returns the current state of the swerve module.
   *
   * @return A {@link SwerveModuleState} object representing the module's current
   *         velocity and angle.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the turning encoder position
    // to get the position relative to the chassis's coordinate system.
    return new SwerveModuleState(m_drivingMotor.getVelocity(),
        new Rotation2d(m_turningMotor.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the swerve module.
   *
   * @return A {@link SwerveModulePosition} object representing the module's total
   *         distance driven and angle.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the turning encoder position
    // to get the position relative to the chassis's coordinate system.
    return new SwerveModulePosition(
        m_drivingMotor.getPosition(),
        new Rotation2d(m_turningMotor.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the swerve module.
   * This method optimizes the angle to prevent unnecessary spinning
   * and then commands the driving and turning motors to achieve the desired speed
   * and angle.
   *
   * @param desiredState The {@link SwerveModuleState} containing the desired
   *                     speed (meters per second)
   *                     and angle (Rotation2d) for the module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // 1. Get the current rotation from the turning encoder
    Rotation2d currentRotation = new Rotation2d(m_turningMotor.getPosition());

    // 2. Optimize the state to avoid spinning more than 90 degrees
    desiredState.optimize(currentRotation);

    // 3. Scale speed by cosine of angle error for smoother driving
    // This prevents the robot from "driving" before the wheels are aligned.
    desiredState.cosineScale(currentRotation);

    // 4. Command the SPARK MAX controllers
    m_drivingMotor.setTargetValue(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningMotor.setTargetValue(desiredState.angle.getRadians(), ControlType.kPosition);
  }

  /** Resets the drive motor's position encoder to zero. */
  public void resetEncoders() {
    m_drivingMotor.setPosition(0);
  }

  public SparkClosedLoopController getDrivingPIDController() {
    return m_drivingMotor.getPIDController();
  }

  public SparkClosedLoopController getTurningPIDController() {
    return m_turningMotor.getPIDController();
  }

  public void setDrivePID(double p, double i, double d)
  {
    this.m_drivingMotor.setPID(p, i, d, 0.0, 0.0);
  }

  public void setDrivePID(double p, double i, double d, double kV)
  {
    this.m_drivingMotor.setPID(p, i, d, kV, 0.0);
  }

  public void setDrivePID(double p, double i, double d, double kV, double kS)
  {
    this.m_drivingMotor.setPID(p, i, d, kV, kS);
  }

  public void setTurnPID(double p, double i, double d, double kV, double kS)
  {
    this.m_turningMotor.setPID(p, i, d, kV, kS);
  }


}