package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static double armTicksInDegrees = 537.7 / 360.0;
    public static double slideTicksInDegrees = 384.5 / 360.0;

    public static double MAX_ARM_POWER = 0.7;
    public static double ARM_INITIAL_ANGLE = 90; //deg
    public static double MAX_SLIDE_POWER_UP = 1;
    public static double MAX_SLIDE_POWER_DOWN = 0.8;
    public static int SLIDE_DEPOSIT_POSITION = 2600;
    public static int SLIDE_SPEC_BAR_POSITION = 1500;
    public static int SLIDE_SPEC_CLIP_POSITION = 950;
    public static int SLIDE_SPEC_GRAB_POSITION = 0;
    public static int ARM_GRAB_POSITION = 880;
    public static int ARM_GRAB_LOWER_POSITION = 910;
    public static int ARM_HOLD_POSITION = 160;
    public static int ARM_TRANSFER_POSITION = 500;
    public static int ARM_SUB_HOLD = 740;
    public static double WRIST_TRANSFER_POSITION = 0.83;
    public static double WRIST_GRAB_POSITION = 0.5;
    public static double WRIST_HOLD_POSITION = 0.4;
    public static double ARM_CLAW_FULL_OPEN = 0.4;
    public static double ARM_CLAW_FULL_CLOSE = 0.68;
    public static double ARM_CLAW_TRANSFER_AND_INTAKE_OPEN = 0.5;
    public static double SPEC_CLAW_OPEN = 0.9;
    public static double SPEC_CLAW_CLOSE = 0.3;
    public static double BUCKET_DEPOSIT_POSITION = 0.78;
    public static double BUCKET_TRANSFER_POSITION = 0.04;
}
