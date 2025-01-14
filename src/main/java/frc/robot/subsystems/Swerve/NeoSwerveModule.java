package frc.robot.subsystems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NeoSwerveModule implements ISwerveModule {

    private final SparkMax mDrivingSparkMax;
    private final SparkMax mTurningSparkMax;

    private final RelativeEncoder mDrivingEncoder;
    private final AbsoluteEncoder mTruningEncoder;

    private final SparkClosedLoopController mDrivingClosedLoopController;
    private final SparkClosedLoopController mTurningClosedLoopController;

    private double mChassisAngularOffset = 0;
    private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());

    @Override
    public SwerveModuleState getState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getState'");
    }

    @Override
    public SwerveModulePosition getPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDesiredState'");
    }

    @Override
    public void resetEncoders() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetEncoders'");
    }
    
}
