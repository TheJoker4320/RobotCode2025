package frc.robot.utils;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.BallCollectorPID;

  
public class Configs {
    public static class BallCollectorConfig {
        public static SparkMaxConfig config = new SparkMaxConfig();
        
        static {
            config.idleMode(IdleMode.kBrake);
            config.smartCurrentLimit(BallCollectorPID.SMART_CURRENT_LIMIT);
            config.absoluteEncoder.positionConversionFactor(BallCollectorPID.ENCODER_POSITION_FACTOR);
            config.absoluteEncoder.velocityConversionFactor(BallCollectorPID.ENCODER_VELOCITY_FACTOR);
            config.absoluteEncoder.inverted(BallCollectorPID.ENCODER_INVERTED);
            config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            config.closedLoop.pid(BallCollectorPID.P_CONSTANT, BallCollectorPID.I_CONSTANT, BallCollectorPID.D_CONSTANT);
            config.closedLoop.outputRange(-1, 1);            
        }

    }
}

