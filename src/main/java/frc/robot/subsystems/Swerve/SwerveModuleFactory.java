package frc.robot.subsystems.Swerve;

import frc.robot.Constants.NeoModuleConstants;

public class SwerveModuleFactory {
    public static ISwerveModule[] generateSwerveModules(SwerveModuleType moduleType) {
        if (moduleType == SwerveModuleType.NEO)
            return generateNeoSwerveModules();
        else
            return null;
    }

    private static ISwerveModule[] generateNeoSwerveModules() {
        ISwerveModule[] swerveModules = new ISwerveModule[NeoModuleConstants.MODULE_COUNT];
        for (int i = 0; i < NeoModuleConstants.MODULE_COUNT; i++) {
            swerveModules[i] = new NeoSwerveModule(
                NeoModuleConstants.DRIVING_CAN_ID[i], 
                NeoModuleConstants.TURNING_CAN_ID[i], 
                i, 
                NeoModuleConstants.ANGULAR_OFFSETS[i]
            );
        }

        return swerveModules;
    }
}
