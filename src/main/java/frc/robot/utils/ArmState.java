package frc.robot.utils;

import frc.robot.Constants.ArmConstants;

public enum ArmState {
    MIN(ArmConstants.ARM_MIN_ANGLE),
    MAX(ArmConstants.ARM_MAX_ANGLE);

    private double m_angle;
    private ArmState(double angle){m_angle = angle;}
    public double angle() {return m_angle;}
}
