package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {
    public static double intakeP = 1;
    public static double intakeI = 0;
    public static double intakeD = 0.005;

    public static double intakeTargetSpeed = 1000;
    public static double intakeOffSpeed = 0;

    public static double shooterP = 0.01;
    public static double shooterI = 0.05; //use integrator (high kI) for high error response
    public static double shooterD = 0;

    public static double targetVelocity = 500;
    public static double offVelocity = 0;

}
