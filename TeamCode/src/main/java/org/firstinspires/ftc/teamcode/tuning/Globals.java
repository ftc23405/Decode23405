package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {
    public static double intakeP = 0.01;
    public static double intakeI = 0;
    public static double intakeD = 0.005;


    public static double intakeTargetSpeed = 1;
    public static double intakeOffSpeed = 0;

    public static double shooterP = 0.003;
    public static double shooterI = 0; //use integrator (high kI) for high error response
    public static double shooterD = 0;
    public static double shooterFF = 0.000195;

    public static double headingP = 0.8;
    public static double headingI = 0.05;

    public static double headingD = 0.04;

    public static double headingFF = 1;

    public static double shooterVelTolerance = 0;

    public static double targetVelocity = 2900;

    public static double classifierAutoVelocity = 2300;
    public static double classifierVelocity = 2400;
    public static double shooterOffVelocity = 0;

    // how many degrees back is your limelight rotated from perfectly vertical?
    public static double limelightMountAngleDegrees = 0;

    // distance from the center of the Limelight lens to the floor
    public static double limelightLensHeightInches = 16.7322;

    // distance from the target to the floor
    public static double goalHeightInches = 29.4094;

    public static double transferPushPosition = 0.6;
    public static double transferHoldPosition = 0.15;

    public static double calculateTicksPerSecond(double targetRPM, double ticksPerRev) {
        return (targetRPM / 60) * ticksPerRev;
    }

    public static double calculateRPM(double tps, double ticksPerRev) {
        return (tps / ticksPerRev) * 60;
    }

}
