package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {
    public static double intakeP = 0.01;
    public static double intakeI = 0;
    public static double intakeD = 0.005;


    public static double intakeTargetSpeed = 1;
    public static double intakeOffSpeed = 0;

    public static double shooterP = 0.01;
    public static double shooterI = 0.001; //use integrator (high kI) for high error response
    public static double shooterD = 0;
    public static double shooterFF = 0.0003;

    public static double shooterVelTolerance = 100;

    public static double targetVelocity = calculateTicksPerSecond(2400, 28);

    public static double classifierAutoVelocity = calculateTicksPerSecond(2300, 28);
    public static double classifierVelocity = calculateTicksPerSecond(2000, 28);
    public static double shooterOffVelocity = 0;



    public static double transferPushPosition = 0.5;
    public static double transferHoldPosition = 0;

    public static double calculateTicksPerSecond(double targetRPM, double ticksPerRev) {
        return (targetRPM / 60) * ticksPerRev;
    }

    public static double calculateRPM(double tps, double ticksPerRev) {
        return (tps / ticksPerRev) * 60;
    }

}
