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
            config.smartCurrentLimit(BallCollectorPID.TURNING_SMART_CURRENT_LIMIT);
            config.absoluteEncoder.positionConversionFactor(BallCollectorPID.TURNING_ENCODER_POSITION_FACTOR);
            config.absoluteEncoder.velocityConversionFactor(BallCollectorPID.TURNING_ENCODER_VELOCITY_FACTOR);
            config.absoluteEncoder.inverted(BallCollectorPID.TURNING_ENCODER_INVERTED);
            config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            config.closedLoop.pid(BallCollectorPID.TURNING_P_CONSTANT, BallCollectorPID.TURNING_I_CONSTANT, BallCollectorPID.TURNING_D_CONSTANT);
            config.closedLoop.outputRange(-1, 1);            
        }

    }
}

