package frc.robot.utils;

import frc.robot.Constants.ArmConstants;

public enum ArmState {
    INTAKE(ArmConstants.INTAKE_ANGLE),
    L1(ArmConstants.L1_ANGLE),
    L32(ArmConstants.L32_ANGLE),
    L4(ArmConstants.L4_ANGLE),
    L32_PLACED(ArmConstants.L32_PLACED_ANGLE),
    L4_PLACED(ArmConstants.L4_PLACED_ANGLE);

    private double mAngle;
    private ArmState(double angle) {mAngle = angle;}
    public double angle() {return mAngle;}
}
