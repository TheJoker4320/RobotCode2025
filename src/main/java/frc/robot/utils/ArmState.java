package frc.robot.utils;

import frc.robot.Constants.ArmConstants;

public enum ArmState {
    INTAKE(ArmConstants.INTAKE_ANGLE),
    OUT_OF_INTAKE(ArmConstants.OUT_OF_INTAKE_ANGLE),
    L1(ArmConstants.L1_ANGLE),
    L32(ArmConstants.L32_ANGLE),
    L4(ArmConstants.L4_ANGLE),
    L32_PLACED(ArmConstants.L32_PLACED_ANGLE),
    L4_PLACED(ArmConstants.L4_PLACED_ANGLE),
    L32_BALL(ArmConstants.L32_BALL_ANGLE),
    ZERO(ArmConstants.ZERO_ANGLE),
    GROUND_BALL_INTAKE(ArmConstants.GROUND_BALL_INTAKE_ANGLE);

    private double mAngle;
    private ArmState(double angle) {mAngle = angle;}
    public double angle() {return mAngle;}
}
