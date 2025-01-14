package frc.robot.subsystems.Swerve;

public class SwerveModuleFactory {
    public static ISwerveModule[] generateSwerveModules(SwerveModuleType moduleType) {
        if (moduleType == SwerveModuleType.NEO)
            return generateNeoSwerveModules();
        else
            return null;
    }

    private static ISwerveModule[] generateNeoSwerveModules() {
        ISwerveModule frontLeft = new NeoSwerveModule(0, 1, 0, 0);
        ISwerveModule frontRight = new NeoSwerveModule(2, 3, 1, 0);
        ISwerveModule rearLeft = new NeoSwerveModule(4, 5, 2, 0);
        ISwerveModule rearRight = new NeoSwerveModule(6, 7, 3, 0);

        return new ISwerveModule[] {frontLeft, frontRight, rearLeft, rearRight};
    }
}
