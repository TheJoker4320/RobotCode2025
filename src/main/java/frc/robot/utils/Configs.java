package frc.robot.utils;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ElevatorConstants;

public final class Configs {
    public static class ElevatorConfigs {
        public static final TalonFXConfiguration ELEVATOR_TALONFX_CONFIG = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = ELEVATOR_TALONFX_CONFIG.Slot0;
            slot0Configs.kP = ElevatorConstants.ELEVATOR_P_CONSTANT;
            slot0Configs.kI = ElevatorConstants.ELEVATOR_I_CONSTANT;
            slot0Configs.kD = ElevatorConstants.ELEVATOR_D_CONSTANT;
            slot0Configs.kG = ElevatorConstants.ELEVATOR_G_CONSTANT;

            SoftwareLimitSwitchConfigs limitSwitchConfigs = ELEVATOR_TALONFX_CONFIG.SoftwareLimitSwitch;
            limitSwitchConfigs.withForwardSoftLimitEnable(ElevatorConstants.MAXIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withForwardSoftLimitThreshold(ElevatorConstants.MAXIMUM_ELEVATOR_HEIGHT);
            limitSwitchConfigs.withForwardSoftLimitEnable(ElevatorConstants.MINIMUM_VALUE_ENABLED);
            limitSwitchConfigs.withForwardSoftLimitThreshold(ElevatorConstants.MINIMUM_ELEVATOR_HEIGHT);

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
}
