//package org.firstinspires.ftc.teamcode.commandbase.subsystems;
//
//import static org.firstinspires.ftc.teamcode.tuning.Globals.*;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.rowanmcalpin.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
//
//import org.firstinspires.ftc.teamcode.tuning.PIDControllerWrapperCustomFF;
//
//public class IntakeArm extends Subsystem {
//
//    public static final IntakeArm INSTANCE = new IntakeArm();
//    private IntakeArm() { }
//
//    public MotorEx intakeArmMotor;
//
//    public String intakeArmMotorName = "intakeArmMotor";
//
//    public PIDControllerWrapperCustomFF intakeArmController = new PIDControllerWrapperCustomFF(new PIDController(0.01, 0.23, 0.0005),0.4, 10);
//
//    public Command toTransfer() {
//        return new RunToPosition(intakeArmMotor,
//                ARM_TRANSFER_POSITION,
//                intakeArmController,
//                this
//        );
//    }
//
//    public Command toGrab() {
//        return new RunToPosition(intakeArmMotor,
//                ARM_GRAB_POSITION,
//                intakeArmController,
//                this
//        );
//    }
//
//    public Command toHold() {
//        return new RunToPosition(intakeArmMotor,
//                ARM_HOLD_POSITION,
//                intakeArmController,
//                this);
//    }
//
//    public Command toSubHold() {
//        return new RunToPosition(intakeArmMotor,
//                ARM_SUB_HOLD,
//                intakeArmController,
//                this);
//    }
//
//    @Override
//    public Command getDefaultCommand() {
//        return new HoldPosition(intakeArmMotor, intakeArmController, this);
//    }
//
//    @Override
//    public void initialize() {
//        intakeArmMotor = new MotorEx(intakeArmMotorName);
//    }
//}
