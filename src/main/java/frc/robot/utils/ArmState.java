package frc.robot.utils;

import frc.robot.Constants.ArmConstants;

public enum ArmState {
    LOW(ArmConstants.ARM_LOW_ANGLE),
    HIGH(ArmConstants.ARM_HIGH_ANGLE);

    private double m_angle;
    private ArmState(double angle){m_angle = angle;}
    public double angle() {return m_angle;}
}
