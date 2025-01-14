package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Configs {
    public static final class NeoSwerveConfigs {
        public static final SparkMaxConfig DRIVING_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig TURNING_CONFIG = new SparkMaxConfig();

        static {
            double DRIVING_ENCODER_CONVERTION_FACTOR = 10; // TODO: Get value from constants
            double TURNING_ENCODER_CONVERTION_FACTOR = 6.28; // TODO: Get value from constants
            double DRIVING_VELOCITY_FEEDFORWARD = 1.0 / 500.0; // TODO: Get value from constants

            DRIVING_CONFIG.idleMode(IdleMode.kBrake);
            DRIVING_CONFIG.smartCurrentLimit(50); // TODO: Get value from constants
            DRIVING_CONFIG.encoder.positionConversionFactor(DRIVING_ENCODER_CONVERTION_FACTOR);
            DRIVING_CONFIG.encoder.velocityConversionFactor(DRIVING_ENCODER_CONVERTION_FACTOR / 60.0);
            DRIVING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            DRIVING_CONFIG.closedLoop.pid(1, 0, 0); // TODO: Get value from constants
            DRIVING_CONFIG.closedLoop.velocityFF(DRIVING_VELOCITY_FEEDFORWARD);
            DRIVING_CONFIG.closedLoop.outputRange(-1, 1);

            TURNING_CONFIG.idleMode(IdleMode.kBrake);
            TURNING_CONFIG.smartCurrentLimit(20); // TODO: Get value from constants
            TURNING_CONFIG.encoder.positionConversionFactor(TURNING_ENCODER_CONVERTION_FACTOR);
            TURNING_CONFIG.encoder.velocityConversionFactor(TURNING_ENCODER_CONVERTION_FACTOR / 60.0);
            TURNING_CONFIG.encoder.inverted(true); // TODO: Get value from constants
            TURNING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            TURNING_CONFIG.closedLoop.pid(1, 0, 0); // TODO: Get value from constants
            TURNING_CONFIG.closedLoop.outputRange(-1, 1);
            TURNING_CONFIG.closedLoop.positionWrappingEnabled(true);
            TURNING_CONFIG.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}
