package frc.robot.subsystems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.Configs.NeoSwerveConfigs;

public class NeoSwerveModule implements ISwerveModule {
    private final int mModuleId;            // For debugging purposes

    private final SparkMax mDrivingSparkMax;
    private final SparkMax mTurningSparkMax;

    private final RelativeEncoder mDrivingEncoder;
    private final AbsoluteEncoder mTurningEncoder;

    private final SparkClosedLoopController mDrivingClosedLoopController;
    private final SparkClosedLoopController mTurningClosedLoopController;

    private double mChassisAngularOffset = 0;
    private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d());

    public NeoSwerveModule(int drivingCANId, int turningCANId, int moduleId, double chassisAngularOffset) {
        mDrivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
        mTurningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

        mDrivingEncoder = mDrivingSparkMax.getEncoder();
        mTurningEncoder = mTurningSparkMax.getAbsoluteEncoder();

        mDrivingClosedLoopController = mDrivingSparkMax.getClosedLoopController();
        mTurningClosedLoopController = mTurningSparkMax.getClosedLoopController();

        mDrivingSparkMax.configure(NeoSwerveConfigs.DRIVING_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mTurningSparkMax.configure(NeoSwerveConfigs.TURNING_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mChassisAngularOffset = chassisAngularOffset;
        mDesiredState.angle = Rotation2d.fromRadians(mTurningEncoder.getPosition());
        mDrivingEncoder.setPosition(0);

        mModuleId = moduleId;
    }

    @Override
    public SwerveModuleState getState() {
        Rotation2d currentModuleRotation = Rotation2d.fromRadians(mTurningEncoder.getPosition()).minus(Rotation2d.fromRadians(mChassisAngularOffset));
        SwerveModuleState currentState = new SwerveModuleState(mDrivingEncoder.getVelocity(), currentModuleRotation);
        return currentState;
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return mDesiredState;
    }

    @Override
    public SwerveModulePosition getPosition() {
        Rotation2d currentModuleRotation = Rotation2d.fromRadians(mTurningEncoder.getPosition()).minus(Rotation2d.fromRadians(mChassisAngularOffset));
        SwerveModulePosition currentPosition = new SwerveModulePosition(mDrivingEncoder.getPosition(), currentModuleRotation);
        return currentPosition;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(mChassisAngularOffset));

        correctedDesiredState.optimize(Rotation2d.fromRadians(mTurningEncoder.getPosition()));

        mDrivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        mTurningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        mDesiredState = desiredState;
    }

    @Override
    public void resetEncoders() {
        mDrivingEncoder.setPosition(0);
    }
    
}
