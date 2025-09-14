//package org.firstinspires.ftc.teamcode.commandbase.subsystems;
//
//import static org.firstinspires.ftc.teamcode.tuning.Globals.*;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.rowanmcalpin.nextftc.core.Subsystem;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
//import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
//import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
//
//import org.firstinspires.ftc.teamcode.tuning.PIDControllerWrapper;
//import org.firstinspires.ftc.teamcode.tuning.PIDFMotorController;
//
//public class Slides extends Subsystem {
//
//    public static final Slides INSTANCE = new Slides();
//    private Slides() { }
//
//    public MotorEx rightSlideMotor;
//
//    public String slideMotorName = "rightSlideMotor";
//
//    public PIDControllerWrapper slidesController = new PIDControllerWrapper(new PIDController(0.01, 0.6, 0.001), 0, 10);
//
//    public Command toLowest() {
//        return new RunToPosition(rightSlideMotor,
//                SLIDE_SPEC_GRAB_POSITION,
//                slidesController,
//                this);
//    }
//
//    public Command toSpecBar() {
//        return new RunToPosition(rightSlideMotor,
//                SLIDE_SPEC_BAR_POSITION,
//                slidesController,
//                this);
//    }
//
//    public Command toSpecClip() {
//        return new RunToPosition(rightSlideMotor,
//                SLIDE_SPEC_CLIP_POSITION,
//                slidesController,
//                this);
//    }
//
//    public Command toDeposit() {
//        return new RunToPosition(rightSlideMotor,
//                SLIDE_DEPOSIT_POSITION,
//                slidesController,
//                this);
//    }
//
//    public Command resetSlidesEncoder() {
//        return new InstantCommand(
//                () -> { slidesController.getFtcLibController().reset(); }
//        );
//    }
//
//    @Override
//    public Command getDefaultCommand() {
//        return new HoldPosition(rightSlideMotor, slidesController, this);
//    }
//
//    @Override
//    public void initialize() {
//        rightSlideMotor = new MotorEx(slideMotorName);
//    }
//
//
//}
