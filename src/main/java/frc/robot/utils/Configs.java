package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.NeoModuleConstants;

public final class Configs {
    public static class NeoSwerveConfigs {
        public static final SparkMaxConfig DRIVING_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig TURNING_CONFIG = new SparkMaxConfig();

        static {
            DRIVING_CONFIG.idleMode(IdleMode.kBrake);
            DRIVING_CONFIG.smartCurrentLimit(NeoModuleConstants.DRIVING_SMART_CURRENT_LIMIT);
            DRIVING_CONFIG.encoder.positionConversionFactor(NeoModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
            DRIVING_CONFIG.encoder.velocityConversionFactor(NeoModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
            DRIVING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            DRIVING_CONFIG.closedLoop.pid(NeoModuleConstants.DRIVING_P_CONSTANT, NeoModuleConstants.DRIVING_I_CONSTANT, NeoModuleConstants.DRIVING_D_CONSTANT);
            DRIVING_CONFIG.closedLoop.velocityFF(NeoModuleConstants.DRIVING_VELOCITY_FF);
            DRIVING_CONFIG.closedLoop.outputRange(-1, 1);

            TURNING_CONFIG.idleMode(IdleMode.kBrake);
            TURNING_CONFIG.smartCurrentLimit(NeoModuleConstants.TURNING_SMART_CURRENT_LIMIT);
            TURNING_CONFIG.encoder.positionConversionFactor(NeoModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
            TURNING_CONFIG.encoder.velocityConversionFactor(NeoModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);
            TURNING_CONFIG.encoder.inverted(NeoModuleConstants.TURNING_ENCODER_INVERTED);
            TURNING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            TURNING_CONFIG.closedLoop.pid(NeoModuleConstants.TURNING_P_CONSTANT, NeoModuleConstants.TURNING_I_CONSTANT, NeoModuleConstants.TURNING_D_CONSTANT);
            TURNING_CONFIG.closedLoop.outputRange(-1, 1);
            TURNING_CONFIG.closedLoop.positionWrappingEnabled(true);
            TURNING_CONFIG.closedLoop.positionWrappingInputRange(0, NeoModuleConstants.MAX_TURNING_ENCODER_VALUE);
        }
    }
}
