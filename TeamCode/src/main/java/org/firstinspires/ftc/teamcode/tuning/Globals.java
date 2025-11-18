package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {
    public static double intakeP = 0.01;
    public static double intakeI = 0;
    public static double intakeD = 0.005;

    public static double intakeTargetSpeed = 1;
    public static double intakeOffSpeed = 0;

    public static double shooterP = 0.011;
    public static double shooterI = 0; //use integrator (high kI) for high error response
    public static double shooterD = 0.0001;
    public static double shooterFF = 0.0005;

    public static double targetVelocity = calculateTicksPerSecond(6000, 28);
    public static double classifierVelocity = calculateTicksPerSecond(3500, 28);
    public static double shooterOffVelocity = 0;

    public static double transferPower = 0.5;
    public static double reverseTransferPower = -0.5;

    public static double calculateTicksPerSecond(double targetRPM, double ticksPerRev) {
        return (targetRPM / 60) * ticksPerRev;
    }

}
