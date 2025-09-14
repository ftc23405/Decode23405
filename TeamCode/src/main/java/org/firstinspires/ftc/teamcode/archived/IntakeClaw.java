//package org.firstinspires.ftc.teamcode.commandbase.subsystems;
//
//import static org.firstinspires.ftc.teamcode.tuning.Globals.*;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.rowanmcalpin.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.ftc.OpModeData;
//import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
//
//public class IntakeClaw extends Subsystem {
//    public static final IntakeClaw INSTANCE = new IntakeClaw();
//    private IntakeClaw() { }
//
//    public Servo clawIntake;
//
//    public String clawIntakeName = "clawIntake";
//
//    public Command full_open() {
//        return new ServoToPosition(clawIntake,
//                ARM_CLAW_FULL_OPEN,
//                this);
//    }
//
//    public Command full_close() {
//        return new ServoToPosition(clawIntake,
//                ARM_CLAW_FULL_CLOSE,
//                this);
//    }
//
//    public Command transfer_intake_open() {
//        return new ServoToPosition(clawIntake,
//                ARM_CLAW_TRANSFER_AND_INTAKE_OPEN,
//                this);
//    }
//
//    @Override
//    public void initialize() {
//        clawIntake = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, clawIntakeName);
//    }
//}
