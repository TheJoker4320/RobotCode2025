package frc.robot.utils;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.BallCollectorConstants;

  
public class Configs {
    public static class BallCollectorConfig {
        public static final SparkMaxConfig ARM_CONFIGS = new SparkMaxConfig();
        public static final SparkMaxConfig COLLECTOR_CONFIGS = new SparkMaxConfig();
        
        static {
            ARM_CONFIGS.idleMode(IdleMode.kBrake);
            ARM_CONFIGS.smartCurrentLimit(BallCollectorConstants.ARM_SMART_CURRENT_LIMIT);
            ARM_CONFIGS.absoluteEncoder.positionConversionFactor(BallCollectorConstants.ENCODER_POSITION_FACTOR);
            ARM_CONFIGS.absoluteEncoder.velocityConversionFactor(BallCollectorConstants.ARM_ENCODER_VELOCITY_FACTOR);
            ARM_CONFIGS.absoluteEncoder.inverted(BallCollectorConstants.ENCODER_INVERTED);
            ARM_CONFIGS.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            ARM_CONFIGS.closedLoop.pid(BallCollectorConstants.P_CONSTANT, BallCollectorConstants.I_CONSTANT, BallCollectorConstants.D_CONSTANT);
            ARM_CONFIGS.closedLoop.outputRange(-1, 1);                  
            COLLECTOR_CONFIGS.idleMode(IdleMode.kBrake);
            COLLECTOR_CONFIGS.smartCurrentLimit(BallCollectorConstants.COLLECTOR_SMART_CURRENT_LIMIT);
        }
    }
}


