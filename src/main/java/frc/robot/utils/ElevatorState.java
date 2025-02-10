package frc.robot.utils;

import frc.robot.Constants.ElevatorConstants;

public enum ElevatorState {
    PRE_INTAKE(ElevatorConstants.PRE_INTAKE_POSITION),
    INTAKE(ElevatorConstants.INTAKE_POSITION),
    L4(ElevatorConstants.L4_POSITION),
    L3(ElevatorConstants.L3_POSITION),
    L2(ElevatorConstants.L2_POSITION),
    L1(ElevatorConstants.L1_POSITION);

    private double mHeight;  
    private ElevatorState(double height) { mHeight = height; }
    public double height() { return mHeight; }
}
