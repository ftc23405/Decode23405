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
//public class DepositBucket extends Subsystem {
//    public static final DepositBucket INSTANCE = new DepositBucket();
//    private DepositBucket() { }
//
//    public Servo bucketServo;
//
//    public String bucketServoName = "bucketServo";
//
//    public Command ToTransfer() {
//        return new ServoToPosition(bucketServo,
//                BUCKET_TRANSFER_POSITION,
//                this);
//    }
//
//
//    public Command ToDeposit() {
//        return new ServoToPosition(bucketServo,
//                BUCKET_DEPOSIT_POSITION,
//                this);
//    }
//
//    @Override
//    public void initialize() {
//        bucketServo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, bucketServoName);
//    }
//}
