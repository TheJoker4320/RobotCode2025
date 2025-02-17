package frc.robot.utils;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

import edu.wpi.first.units.Units;
import frc.robot.Constants.NeoModuleConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.ArmConstants;

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
            // Note that we are using motion magic
            slot0Configs.kV = ElevatorConstants.ELEVATOR_V_CONSTANT;
            slot0Configs.kA = ElevatorConstants.ELEVATOR_A_CONSTANT;
            slot0Configs.kS = ElevatorConstants.ELEVATOR_S_CONSTANT;

            MotionMagicConfigs motionMagicConfigs = ELEVATOR_TALONFX_CONFIG.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MM_CRUISE_VELOCITY;
            motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MM_ACCELERATION;
            motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MM_JERK;

            SoftwareLimitSwitchConfigs limitSwitchConfigs = ELEVATOR_TALONFX_CONFIG.SoftwareLimitSwitch;
            limitSwitchConfigs.withForwardSoftLimitEnable(ElevatorConstants.MAXIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withForwardSoftLimitThreshold(Rotations.of(ElevatorConstants.MAXIMUM_ELEVATOR_HEIGHT));
            limitSwitchConfigs.withReverseSoftLimitEnable(ElevatorConstants.MINIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withReverseSoftLimitThreshold(Rotations.of(ElevatorConstants.MINIMUM_ELEVATOR_HEIGHT));

            FeedbackConfigs feedbackConfigs = ELEVATOR_TALONFX_CONFIG.Feedback;
            feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            feedbackConfigs.SensorToMechanismRatio = ElevatorConstants.ELV_SENSOR_TO_MECAHNISM_RATIO;

            MotorOutputConfigs motorOutputConfigs = ELEVATOR_TALONFX_CONFIG.MotorOutput;
            motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
            motorOutputConfigs.Inverted = ElevatorConstants.RIGHT_MOTOR_INVERTED;

            CurrentLimitsConfigs currentLimitsConfigs = ELEVATOR_TALONFX_CONFIG.CurrentLimits;
            currentLimitsConfigs.withStatorCurrentLimitEnable(ElevatorConstants.CURRENT_LIMIT_ENABLED);
            currentLimitsConfigs.withStatorCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
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

    public static class ManipulatorConfigs {
        public static final SparkMaxConfig BALL_COLLECTOR_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig CORAL_COLLECTOR_CONFIG = new SparkMaxConfig();

        static {
            BALL_COLLECTOR_CONFIG.idleMode(IdleMode.kBrake);
            BALL_COLLECTOR_CONFIG.smartCurrentLimit(ManipulatorConstants.MANIPULATOR_BALL_SMART_CURRENT_LIMIT);
            BALL_COLLECTOR_CONFIG.inverted(ManipulatorConstants.BALL_COLLECTOR_INVERTED);
            CORAL_COLLECTOR_CONFIG.idleMode(IdleMode.kBrake);
            CORAL_COLLECTOR_CONFIG.smartCurrentLimit(ManipulatorConstants.MANIPULATOR_BALL_SMART_CURRENT_LIMIT);
        }
    }
   public static class ArmConfigs {
        public static TalonFXConfiguration ARM_TALONFX_CONFIG = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = ARM_TALONFX_CONFIG.Slot0;
            slot0Configs.kP = ArmConstants.ARM_KP;
            slot0Configs.kI = ArmConstants.ARM_KI;
            slot0Configs.kD = ArmConstants.ARM_KD;
            slot0Configs.kG = ArmConstants.ARM_KG;
            slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

            MotorOutputConfigs motorOutputConfigs = ARM_TALONFX_CONFIG.MotorOutput;
            motorOutputConfigs.Inverted = ArmConstants.INVERTED_VALUE;

            SoftwareLimitSwitchConfigs limitSwitchConfigs = ARM_TALONFX_CONFIG.SoftwareLimitSwitch;
            limitSwitchConfigs.withForwardSoftLimitEnable(ArmConstants.MAXIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withForwardSoftLimitThreshold(Units.Degree.of(ArmConstants.MAXIMUM_ARM_ANGLE));
            limitSwitchConfigs.withReverseSoftLimitEnable(ArmConstants.MINIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withReverseSoftLimitThreshold(Units.Degree.of(ArmConstants.MINIMUM_ARM_ANGLE));
            
            FeedbackConfigs feedback = ARM_TALONFX_CONFIG.Feedback;
            feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            feedback.SensorToMechanismRatio = ArmConstants.MOTOR_TO_ARM_GEAR_RATIO;

            MotorOutputConfigs motorOutput = ARM_TALONFX_CONFIG.MotorOutput;
            motorOutput.NeutralMode = NeutralModeValue.Brake;

            CurrentLimitsConfigs currentLimit = ARM_TALONFX_CONFIG.CurrentLimits;
            currentLimit.StatorCurrentLimitEnable = ArmConstants.SMART_CURRENT_LIMIT_ENABLED;
            currentLimit.StatorCurrentLimit = ArmConstants.SMART_CURRENT_LIMIT;
             
            //this code uses motion magic for the arm
            if (ArmConstants.IS_MAGIC_MOTION_ENABLED) {
                slot0Configs.kS = ArmConstants.ARM_KS;
                slot0Configs.kV = ArmConstants.ARM_KV;
                slot0Configs.kA = ArmConstants.ARM_KA;

                MotionMagicConfigs motionMagicConfigs = ARM_TALONFX_CONFIG.MotionMagic;
                motionMagicConfigs.withMotionMagicCruiseVelocity(DegreesPerSecond.of(ArmConstants.MM_CRUISE_VELOCITY));
                motionMagicConfigs.withMotionMagicAcceleration(DegreesPerSecondPerSecond.of(ArmConstants.MM_ACCELERATION));
                motionMagicConfigs.withMotionMagicJerk(DegreesPerSecondPerSecond.per(Second).of(ArmConstants.MM_JERK));            // Optional
            }
        }
    }
}
