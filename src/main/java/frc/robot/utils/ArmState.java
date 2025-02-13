package frc.robot.utils;

import frc.robot.Constants.ArmConstants;

public enum ArmState {
    LOW(ArmConstants.ARM_LOW_ANGLE),
    HIGH(ArmConstants.ARM_HIGH_ANGLE);

    private double mAngle;
    private ArmState(double angle) {mAngle = angle;}
    public double angle() {return mAngle;}
}
