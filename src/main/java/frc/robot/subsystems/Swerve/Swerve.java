package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveSubsystemConstants;

public class Swerve extends SubsystemBase {

    private double mInputMultiplier = 1;

    private Pigeon2 mGyro;

    private ISwerveModule mFrontLeft;
    private ISwerveModule mFrontRight;
    private ISwerveModule mRearLeft;
    private ISwerveModule mRearRight;

    private SwerveDriveOdometry mOdometry;

    private static Swerve mInstance = null;
    public Swerve getInstance(SwerveModuleType moduleType) {
        if (mInstance == null)
            mInstance = new Swerve(moduleType);
        return mInstance;
    }

    private Swerve(SwerveModuleType moduleType) {
        ISwerveModule[] swerveModules = SwerveModuleFactory.generateSwerveModules(moduleType);
        mFrontLeft = swerveModules[0];
        mFrontRight = swerveModules[1];
        mRearLeft = swerveModules[2];
        mRearRight = swerveModules[3];

        mGyro = new Pigeon2(SwerveSubsystemConstants.PIGEON_DEVICE_ID);                             
        mGyro.getConfigurator().apply(new Pigeon2Configuration());
        mGyro.setYaw(0);

        mOdometry = new SwerveDriveOdometry(
            SwerveSubsystemConstants.DRIVE_KINEMATICS,                                                                        
            getRotation(),
            getModulePositions()
        );
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean robotFieldRelative) {
        double xSpeedCommand = xSpeed * mInputMultiplier;
        double ySpeedCommand = ySpeed * mInputMultiplier;
        double rotCommand = rot * mInputMultiplier;

        double xSpeedDelivered = xSpeedCommand * SwerveSubsystemConstants.MAX_SPEED;                
        double ySpeedDelivered = ySpeedCommand * SwerveSubsystemConstants.MAX_SPEED;                
        double rotDelivered = rotCommand * SwerveSubsystemConstants.MAX_ANGULAR_SPEED;              

        SwerveModuleState[] swerveModuleStates;
        if (robotFieldRelative) {
            double xSpeedAdjusted = xSpeedDelivered * Math.cos(getRotation().getRadians() * -1) + ySpeedDelivered *  Math.sin(getRotation().getRadians() * -1);
            double ySpeedAdjusted = -1 * xSpeedDelivered * Math.sin(getRotation().getRadians() * -1) + ySpeedDelivered *  Math.cos(getRotation().getRadians() * -1);

            swerveModuleStates = SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotDelivered));
        } else {
            swerveModuleStates = SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        }

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSubsystemConstants.MAX_SPEED);
        mFrontLeft.setDesiredState(desiredStates[0]);
        mFrontRight.setDesiredState(desiredStates[1]);
        mRearLeft.setDesiredState(desiredStates[2]);
        mRearRight.setDesiredState(desiredStates[3]);
    }

    public void setX() {
        SwerveModuleState[] xModuleStates = {
            new SwerveModuleState(0, Rotation2d.fromRadians(SwerveSubsystemConstants.X_STATE_ANGLE)),
            new SwerveModuleState(0, Rotation2d.fromRadians(-SwerveSubsystemConstants.X_STATE_ANGLE)),
            new SwerveModuleState(0, Rotation2d.fromRadians(-SwerveSubsystemConstants.X_STATE_ANGLE)),
            new SwerveModuleState(0, Rotation2d.fromRadians(SwerveSubsystemConstants.X_STATE_ANGLE))
        };
        setModuleStates(xModuleStates);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] SwerveModulePositions = {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mRearLeft.getPosition(),
            mRearRight.getPosition()
        };

        return SwerveModulePositions;
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(mGyro.getYaw().getValue().magnitude());
    }

    public void zeroHeading() {
        mGyro.reset();
        resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public void setInputMultiplier(double inputMultiplier) {
        mInputMultiplier = inputMultiplier;
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }
    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        mOdometry.update(
            getRotation(), 
            getModulePositions()
        );
    }
}
