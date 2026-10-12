// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.LauncherConstants;

/** Add your docs here. */
public class LauncherConfigs {
    public static final SparkMaxConfig config = new SparkMaxConfig();
    
    static {
        config.encoder
            .positionConversionFactor(LauncherConstants.kPositionFactor)
            .velocityConversionFactor(LauncherConstants.kPositionFactor);

        config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(LauncherConstants.kLauncherP.get(), LauncherConstants.kLauncherI.get(), LauncherConstants.kLauncherD.get())
        // Never drive the flywheel backwards. Slowing down from shot speed would
        // otherwise reverse-brake the wheel, pulling a big current spike.
        .outputRange(0, 1);
        
        config.closedLoop.feedForward
        .kV(LauncherConstants.kLauncherkV.get())
        .kS(LauncherConstants.kLauncherStatic.get());

        config.inverted(false);        
        config.idleMode(IdleMode.kCoast); 
        config.smartCurrentLimit(GlobalConstants.kMediumCurrentLimit);
    }
}
