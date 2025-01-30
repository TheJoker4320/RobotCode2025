package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.OperatorConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Configs {
    public static class ManipulatorConfigs {
        public static final SparkMaxConfig BALL_COLLECTOR_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig CORAL_COLLECTOR_CONFIG = new SparkMaxConfig();

        static {
            BALL_COLLECTOR_CONFIG.idleMode(IdleMode.kBrake);
            BALL_COLLECTOR_CONFIG.smartCurrentLimit(OperatorConstants.MANIPULATOR_BALL_SMART_CURRENT_LIMIT);
            CORAL_COLLECTOR_CONFIG.idleMode(IdleMode.kBrake);
            CORAL_COLLECTOR_CONFIG.smartCurrentLimit(OperatorConstants.MANIPULATOR_BALL_SMART_CURRENT_LIMIT);
        }
    }
}
