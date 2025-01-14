package frc.robot.utils;

public enum ElevatorState {
    LOW(0),                 // TODO: Get value from constants
    HIGH(100);              // TODO: Get value from constants

    private double mHeight;        // Notice that the value is in rotations         
    private ElevatorState(double height) { mHeight = height; }
    public double height() { return mHeight; }
}
