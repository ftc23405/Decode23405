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
//public class SpecClaw extends Subsystem {
//
//    public static final SpecClaw INSTANCE = new SpecClaw();
//    private SpecClaw() { }
//
//    public Servo specServo;
//
//    public String specServoName = "specServo";
//
//    public Command open() {
//        return new ServoToPosition(specServo,
//                SPEC_CLAW_OPEN,
//                this);
//    }
//
//    public Command close() {
//        return new ServoToPosition(specServo,
//                SPEC_CLAW_CLOSE,
//                this);
//    }
//
//    @Override
//    public void initialize() {
//        specServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, specServoName);
//    }
//}
