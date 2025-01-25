package frc.robot.utils;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.BallCollectorConstants;

  
public class Configs {
    public static class BallCollectorConfig {
        public static SparkMaxConfig armConfigs = new SparkMaxConfig();
        public static SparkMaxConfig collectorCofigs = new SparkMaxConfig();
        
        static {
            armConfigs.idleMode(IdleMode.kBrake);
            armConfigs.smartCurrentLimit(BallCollectorConstants.ARM_SMART_CURRENT_LIMIT);
            armConfigs.absoluteEncoder.positionConversionFactor(BallCollectorConstants.ENCODER_POSITION_FACTOR);
            armConfigs.absoluteEncoder.velocityConversionFactor(BallCollectorConstants.ARM_ENCODER_VELOCITY_FACTOR);
            armConfigs.absoluteEncoder.inverted(BallCollectorConstants.ENCODER_INVERTED);
            armConfigs.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            armConfigs.closedLoop.pid(BallCollectorConstants.P_CONSTANT, BallCollectorConstants.I_CONSTANT, BallCollectorConstants.D_CONSTANT);
            armConfigs.closedLoop.outputRange(-1, 1);        
            
            collectorCofigs.idleMode(IdleMode.kBrake);
            collectorCofigs.smartCurrentLimit(BallCollectorConstants.COLLECTOR_SMART_CURRENT_LIMIT);
        }


    }
}

