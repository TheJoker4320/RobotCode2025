package frc.robot.utils;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class Configs {
    public static class ArmConfigs{
        public static TalonFXConfiguration ARM_TALONFX_CONFIG = new TalonFXConfiguration();

        static{
            Slot0Configs slot0Configs = ARM_TALONFX_CONFIG.Slot0;
            slot0Configs.kP = ArmConstants.ARM_KP;
            slot0Configs.kI = ArmConstants.ARM_KI;
            slot0Configs.kD = ArmConstants.ARM_KD;
            slot0Configs.kG = ArmConstants.ARM_KG;

            SoftwareLimitSwitchConfigs limitSwitchConfigs = ARM_TALONFX_CONFIG.SoftwareLimitSwitch;
            limitSwitchConfigs.withForwardSoftLimitEnable(true);
            limitSwitchConfigs.withForwardSoftLimitThreshold(100);

             /*
             * This code is for when we want to add motion magic to the arm
             * Notice that when you calculate these values (probably using ReCalc) you recive it in radians - instead of radians you want
             * it to be rotation - so you MUST convert the meters to rotation either from:
             * the formula given in constants
             * the conversion given in ReCalc
             */
            if (ArmConstants.IS_MAGIC_MOTION_ENABLED){
                slot0Configs.kV = ArmConstants.ARM_KV;
                slot0Configs.kA = ArmConstants.ARM_KA;

                MotionMagicConfigs motionMagicConfigs = ARM_TALONFX_CONFIG.MotionMagic;
                motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.MM_CRUISE_VELOCITY;
                motionMagicConfigs.MotionMagicAcceleration = ArmConstants.MM_ACCELERATION;
                //optional
                motionMagicConfigs.MotionMagicJerk = ArmConstants.MM_JERK;
            }
        }
    }
}
