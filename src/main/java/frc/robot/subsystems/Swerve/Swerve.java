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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveSubsystemConstants;

public class Swerve extends SubsystemBase {

    private double mInputMultiplier = SwerveSubsystemConstants.REGULAR_INPUT_MULTIPLIER;

    private Pigeon2 mGyro;

    private ISwerveModule mFrontLeft;
    private ISwerveModule mFrontRight;
    private ISwerveModule mRearLeft;
    private ISwerveModule mRearRight;

    private SwerveDriveOdometry mOdometry;

    private Boolean mFieldRelative;

    private StructArrayPublisher<SwerveModuleState> mSwerveStatesPublisher;
    private StructArrayPublisher<SwerveModuleState> mDesiredSwerveStatesPublisher;

    private StructArrayLogEntry<SwerveModuleState> mSwerveStatesLogEntry;
    private StructArrayLogEntry<SwerveModuleState> mDesiredSwerveStatesLogEntry;

    private static Swerve mInstance = null;
    public static Swerve getInstance(SwerveModuleType moduleType) {
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
        mGyro.setYaw(180);

        mOdometry = new SwerveDriveOdometry(
            SwerveSubsystemConstants.DRIVE_KINEMATICS,                                                                        
            getRotation(),
            getModulePositions()
        );

        mFieldRelative = true;

        mSwerveStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveStates", SwerveModuleState.struct).publish();
        mDesiredSwerveStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("DesiredSwerveStates", SwerveModuleState.struct).publish();

        DataLog log = DataLogManager.getLog();
        mSwerveStatesLogEntry = StructArrayLogEntry.create(log, "/joker/swerve/moduleStates", SwerveModuleState.struct);
        mDesiredSwerveStatesLogEntry = StructArrayLogEntry.create(log, "/joker/swerve/desiredStates", SwerveModuleState.struct);
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        SmartDashboard.putNumber("Mult", mInputMultiplier);
        double xSpeedCommand = xSpeed * mInputMultiplier;
        double ySpeedCommand = ySpeed * mInputMultiplier;
        double rotCommand = rot * mInputMultiplier;

        double xSpeedDelivered = xSpeedCommand * SwerveSubsystemConstants.MAX_SPEED;                
        double ySpeedDelivered = ySpeedCommand * SwerveSubsystemConstants.MAX_SPEED;                
        double rotDelivered = rotCommand * SwerveSubsystemConstants.MAX_ANGULAR_SPEED;              

        SwerveModuleState[] swerveModuleStates;
        if (mFieldRelative) {
            double xSpeedAdjusted = xSpeedDelivered * Math.cos(getRotation().getRadians()) + ySpeedDelivered *  Math.sin(getRotation().getRadians());
            double ySpeedAdjusted = -1 * xSpeedDelivered * Math.sin(getRotation().getRadians()) + ySpeedDelivered *  Math.cos(getRotation().getRadians());

            swerveModuleStates = SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeedAdjusted, ySpeedAdjusted, rotDelivered));
        } else {
            swerveModuleStates = SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        }

        setModuleStates(swerveModuleStates);
    }

    // Switches to what the robot is relative to, AKA:
    // Robot-Relative <---> Field-Relative
    public void switchReferenceFrame() {
        mFieldRelative = !mFieldRelative;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveSubsystemConstants.MAX_SPEED);
        mFrontLeft.setDesiredState(desiredStates[0]);
        mFrontRight.setDesiredState(desiredStates[1]);
        mRearLeft.setDesiredState(desiredStates[2]);
        mRearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * This method makes the swerve module form an "X" (causes them all to look at the center)
     * this will result in the robot locking in place - shouldn't be use as an emergency brake
     */
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

    public double getAngularVelocity() {
        return mGyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    public void resetHeading() {
        resetHeading(0);
    }

    public void setInputMultiplier(double inputMultiplier) {
        mInputMultiplier = inputMultiplier;
    }

    public void resetHeading(double newYaw) {
        mGyro.setYaw(newYaw);
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(newYaw)));
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }
    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveSubsystemConstants.DRIVE_KINEMATICS.toChassisSpeeds(
            mFrontLeft.getState(),
            mFrontRight.getState(),
            mRearLeft.getState(),
            mRearRight.getState()
        );
    }

    private void logDesiredSwerveStates() {
        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        desiredStates[0] = mFrontLeft.getDesiredState();
        desiredStates[1] = mFrontRight.getDesiredState();
        desiredStates[2] = mRearLeft.getDesiredState();
        desiredStates[3] = mRearRight.getDesiredState();

        mDesiredSwerveStatesPublisher.set(desiredStates);
        mDesiredSwerveStatesLogEntry.append(desiredStates);
    }

    private void logSwerveStates() {
        SwerveModuleState[] currentStates = new SwerveModuleState[4];
        currentStates[0] = mFrontLeft.getState();
        currentStates[1] = mFrontRight.getState();
        currentStates[2] = mRearLeft.getState();
        currentStates[3] = mRearRight.getState();

        mSwerveStatesPublisher.set(currentStates);
        mSwerveStatesLogEntry.append(currentStates);
    }

    @Override
    public void periodic() {
        mOdometry.update(
            getRotation(), 
            getModulePositions()
        );

        logDesiredSwerveStates();
        logSwerveStates();
    }
}
