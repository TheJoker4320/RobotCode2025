package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.ArmConstants;

public class Configs {
    public static class ArmConfigs{
        public static TalonFXConfiguration ARM_TALONFX_CONFIG = new TalonFXConfiguration();

        static{
            Slot0Configs slot0Configs = ARM_TALONFX_CONFIG.Slot0;
            slot0Configs.kP = ArmConstants.ARM_KP;
            slot0Configs.kI = ArmConstants.ARM_KI;
            slot0Configs.kD = ArmConstants.ARM_KD;
            slot0Configs.kG = ArmConstants.ARM_KG;

             /*
             * This code is for when we want to add motion magic to the elevator
             * Notice that when you calculate these values (probably using ReCalc) you recive it in meters - instead of meters you want
             * it to be rotation - so you MUST convert the meters to rotations
             * 
             * slot0Configs.kV = AemConstants.ARM_V_CONSTANT;
             * slot0Configs.kA = AemConstants.ARM_A_CONSTANT;
             * 
             * MotionMagicConfigs motionMagicConfigs = ARM_TALONFX_CONFIG.MotionMagic;
             * motionMagicConfigs.MotionMagicCruiseVelocity = AemConstants.MM_CRUISE_VELOCITY;
             * motionMagicConfigs.MotionMagicAcceleration = AemConstants.MM_ACCELERATION;
             * (optional) motionMagicConfigs.MotionMagicJerk = AemConstants.MM_JERK;
             */
        }
    }
}
