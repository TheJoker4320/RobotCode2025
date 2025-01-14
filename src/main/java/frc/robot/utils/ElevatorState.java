package frc.robot.utils;

import frc.robot.Constants.ElevatorConstants;

public enum ElevatorState {
    LOW(ElevatorConstants.LOW_POSITION_HEIGHT),
    HIGH(ElevatorConstants.HIGH_POSITION_HEIGHT);

    private double mHeight;        // Notice that the value is in rotations         
    private ElevatorState(double height) { mHeight = height; }
    public double height() { return mHeight; }
}
