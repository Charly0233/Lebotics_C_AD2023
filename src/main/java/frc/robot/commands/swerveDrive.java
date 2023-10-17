package frc.robot.commands;

import frc.robot.subsystems.serveWheel;

public class swerveDrive {
        //public final  double L = 21; // length of robot between left and right axles
    //public final  double W = L; // width, but W and L are equal

    ///change this to the values for the robot
    public final double L = 0.5334/2;  // in meters
    public final double W = 0.5334/2;  // in meters

    private serveWheel backRight;
    private serveWheel frontRight;
    private serveWheel backLeft;
    private serveWheel frontLeft;

    public swerveDrive(serveWheel backRight, serveWheel frontRight, serveWheel backLeft,serveWheel frontLeft){
        this.backLeft=backLeft;
        this.frontLeft=frontLeft;
        this.backRight=backRight;
        this.frontRight=frontRight;
    }

    public void drive(double speed, souble turn, )
}
