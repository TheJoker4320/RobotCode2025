package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ISwerveModule {
    public SwerveModuleState getState();
    public SwerveModulePosition getPosition();
    public void setDesiredState(SwerveModuleState state);
    public void resetEncoders();
    public void displayData();
}
