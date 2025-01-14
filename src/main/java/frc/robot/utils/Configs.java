package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
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
        }
    }
}
