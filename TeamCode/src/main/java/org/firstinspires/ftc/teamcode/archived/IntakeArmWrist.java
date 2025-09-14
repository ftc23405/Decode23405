//package org.firstinspires.ftc.teamcode.commandbase.subsystems;
//
//
//import static org.firstinspires.ftc.teamcode.tuning.Globals.*;
//
//import com.qualcomm.robotcore.hardware.Servo;
//import com.rowanmcalpin.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.ftc.OpModeData;
//import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
//
//public class IntakeArmWrist extends Subsystem {
//
//    public static final IntakeArmWrist INSTANCE = new IntakeArmWrist();
//    private IntakeArmWrist() { }
//
//    public Servo rightWristServo;
//
//    public String rightWristServoName = "rightWristServo";
//
//    public Command hold() {
//        return new ServoToPosition(rightWristServo,
//                WRIST_HOLD_POSITION,
//                this);
//    }
//
//    public Command grab() {
//        return new ServoToPosition(rightWristServo,
//                WRIST_GRAB_POSITION,
//                this);
//    }
//
//    public Command transfer() {
//        return new ServoToPosition(rightWristServo,
//                WRIST_TRANSFER_POSITION,
//                this);
//    }
//
//    @Override
//    public void initialize() {
//        rightWristServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, rightWristServoName);
//    }
//}
