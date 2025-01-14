package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class Configs {
    public static class ElevatorConfigs {
        public static final TalonFXConfiguration ELEVATOR_TALONFX_CONFIG = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = ELEVATOR_TALONFX_CONFIG.Slot0;
            slot0Configs.kP = 1;            // TODO: Get value from constants
            slot0Configs.kI = 0;            // TODO: Get value from constants
            slot0Configs.kD = 0;            // TODO: Get value from constants
        }
    }
}
