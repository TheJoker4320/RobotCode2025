package frc.robot.utils;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.NeoModuleConstants;
import frc.robot.Constants.ElevatorConstants;

public final class Configs {
    public static class ElevatorConfigs {
        public static final TalonFXConfiguration ELEVATOR_TALONFX_CONFIG = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = ELEVATOR_TALONFX_CONFIG.Slot0;
            slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
            slot0Configs.kP = ElevatorConstants.ELEVATOR_P_CONSTANT;
            slot0Configs.kI = ElevatorConstants.ELEVATOR_I_CONSTANT;
            slot0Configs.kD = ElevatorConstants.ELEVATOR_D_CONSTANT;
            slot0Configs.kG = ElevatorConstants.ELEVATOR_G_CONSTANT;

            SoftwareLimitSwitchConfigs limitSwitchConfigs = ELEVATOR_TALONFX_CONFIG.SoftwareLimitSwitch;
            limitSwitchConfigs.withForwardSoftLimitEnable(ElevatorConstants.MAXIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withForwardSoftLimitThreshold(ElevatorConstants.MAXIMUM_ELEVATOR_HEIGHT);
            limitSwitchConfigs.withForwardSoftLimitEnable(ElevatorConstants.MINIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withForwardSoftLimitThreshold(ElevatorConstants.MINIMUM_ELEVATOR_HEIGHT);

            FeedbackConfigs feedbackConfigs = ELEVATOR_TALONFX_CONFIG.Feedback;
            feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            feedbackConfigs.SensorToMechanismRatio = ElevatorConstants.MOTOR_ROTATION_TO_HEIGHT_FACTOR;

            MotorOutputConfigs motorOutputConfigs = ELEVATOR_TALONFX_CONFIG.MotorOutput;
            motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

            /*
             * This code is for when we want to add motion magic to the elevator
             * Notice that when you calculate these values (probably using ReCalc) you recive it in meters - instead of meters you want
             * it to be rotation - so you MUST convert the meters to rotations
             */
            if (ElevatorConstants.MOTIONMAGIC_ENABLED) {
                slot0Configs.kV = ElevatorConstants.ELEVATOR_V_CONSTANT;
                slot0Configs.kA = ElevatorConstants.ELEVATOR_A_CONSTANT;

                MotionMagicConfigs motionMagicConfigs = ELEVATOR_TALONFX_CONFIG.MotionMagic;
                motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELOCITY;
                motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION;
                motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MM_JERK;         // Optional
            }
        }
    }

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
            TURNING_CONFIG.absoluteEncoder.positionConversionFactor(NeoModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
            TURNING_CONFIG.absoluteEncoder.velocityConversionFactor(NeoModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);
            TURNING_CONFIG.absoluteEncoder.inverted(NeoModuleConstants.TURNING_ENCODER_INVERTED);
            TURNING_CONFIG.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            TURNING_CONFIG.closedLoop.pid(NeoModuleConstants.TURNING_P_CONSTANT, NeoModuleConstants.TURNING_I_CONSTANT, NeoModuleConstants.TURNING_D_CONSTANT);
            TURNING_CONFIG.closedLoop.outputRange(-1, 1);
            TURNING_CONFIG.closedLoop.positionWrappingEnabled(true);
            TURNING_CONFIG.closedLoop.positionWrappingInputRange(0, NeoModuleConstants.MAX_TURNING_ENCODER_VALUE);
        }
    }
}
