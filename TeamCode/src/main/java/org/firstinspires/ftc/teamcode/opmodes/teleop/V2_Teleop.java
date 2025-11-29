//package org.firstinspires.ftc.teamcode.opmodes.teleop;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
//import org.firstinspires.ftc.teamcode.commandbase.vision.AprilTagWebcam;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import dev.nextftc.bindings.BindingManager;
//import dev.nextftc.bindings.Button;
//import dev.nextftc.core.commands.delays.Delay;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.components.BindingsComponent;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.core.units.Angle;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.extensions.pedro.PedroDriverControlled;
//import dev.nextftc.extensions.pedro.TurnBy;
//import dev.nextftc.ftc.ActiveOpMode;
//import dev.nextftc.ftc.Gamepads;
//import dev.nextftc.ftc.NextFTCOpMode;
//import dev.nextftc.ftc.components.BulkReadComponent;
//
//@TeleOp
//@Configurable //panels
//public class V2_Teleop extends NextFTCOpMode {
//
//    public V2_Teleop() {
//        addComponents( //add needed components
//                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
//                new SubsystemComponent(TransferPusher.INSTANCE),
//                BulkReadComponent.INSTANCE,
//                BindingsComponent.INSTANCE,
//                new PedroComponent(Constants::createFollower) //follower for Pedro teleop drive
//        );
//    }
//    private final Pose parkingPose = new Pose(38.7,33.2, Math.toRadians(90));
//
//    private TelemetryManager telemetryM;
//
//    private PathChain parkPath;
//
//    private AprilTagWebcam webcam;
//
//    Button parkButton = (Gamepads.gamepad1().dpadUp()).and(Gamepads.gamepad2().dpadUp());
//
//
//    @Override
//    public void onInit() {
//        webcam.initalize(hardwareMap, telemetryM);
//        PedroComponent.follower().setStartingPose(new Pose(0,0, Math.toRadians(180))); //set starting pose for pinpoint IMU
//
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//
//        PedroDriverControlled driverControlled = new PedroDriverControlled(
//                Gamepads.gamepad1().leftStickY(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX().negate(), //negate the right stick because it's inverted
//                false
//        );
//        driverControlled.schedule();
//
//        Gamepads.gamepad1().leftBumper()
//                .whenBecomesTrue(new SequentialGroup(
//                        new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180)))),
//                        new InstantCommand(() -> gamepad1.rumble(500))
//                )); //reset pinpoint IMU
//
//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(() -> webcam.start())
//                .whenTrue(new SequentialGroup(
//                        new InstantCommand(() -> webcam.update()),
//                        new TurnBy(Angle.fromDeg(webcam.getTagBearing(24))))
//                )
//                        .whenFalse(() -> webcam.pause()); // stop streaming to save CPU
//
//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed)
//                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
//        Gamepads.gamepad1().x()
//                .whenBecomesTrue(Intake.INSTANCE.intakeHalfSpeed)
//                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(Intake.INSTANCE.intakeOneThirdSpeed)
//                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
//        Gamepads.gamepad1().b()
//                .whenBecomesTrue(Intake.INSTANCE.intakeReverseFullSpeed)
//                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
//
//        Gamepads.gamepad2().dpadLeft()
//                .whenBecomesTrue(new SequentialGroup(
//                        Shooter.INSTANCE.shooterCloseShoot,
//                        new Delay(0.75),
//                        Intake.INSTANCE.intakeFullSpeed,
//                        TransferPusher.INSTANCE.transferPush,
//                        new Delay(0.15),
//                        TransferPusher.INSTANCE.transferHold,
//                        new Delay(0.25),
//                        TransferPusher.INSTANCE.transferPush,
//                        new Delay(0.15),
//                        TransferPusher.INSTANCE.transferHold,
//                        new Delay(0.25),
//                        TransferPusher.INSTANCE.transferPush
//                ))
//                .whenBecomesFalse(Intake.INSTANCE.intakeOff)
//                .whenBecomesFalse(TransferPusher.INSTANCE.transferHold);
//
//        Gamepads.gamepad2().dpadUp()
//                .whenBecomesTrue(new SequentialGroup(
//                        Shooter.INSTANCE.shooterFarShoot,
//                        new Delay(0.75),
//                        Intake.INSTANCE.intakeFullSpeed,
//                        TransferPusher.INSTANCE.transferPush,
//                        new Delay(0.15),
//                        TransferPusher.INSTANCE.transferHold,
//                        new Delay(0.25),
//                        TransferPusher.INSTANCE.transferPush,
//                        new Delay(0.15),
//                        TransferPusher.INSTANCE.transferHold,
//                        new Delay(0.25),
//                        TransferPusher.INSTANCE.transferPush
//                ))
//                .whenBecomesFalse(Intake.INSTANCE.intakeOff)
//                .whenBecomesFalse(TransferPusher.INSTANCE.transferHold);
//        Gamepads.gamepad2().rightBumper()
//                .whenBecomesTrue(Shooter.INSTANCE.shooterFarShoot);
////                .whenBecomesFalse(new SequentialGroup(
////                        new Delay(0.75),
////                        Intake.INSTANCE.intakeFullSpeed,
////                        TransferPusher.INSTANCE.transferPush,
////                        new Delay(0.15),
////                        TransferPusher.INSTANCE.transferHold,
////                        new Delay(0.25),
////                        TransferPusher.INSTANCE.transferPush,
////                        new Delay(0.15),
////                        TransferPusher.INSTANCE.transferHold,
////                        new Delay(0.25),
////                        TransferPusher.INSTANCE.transferPush
////                ));
//        Gamepads.gamepad2().leftBumper()
//                .whenBecomesTrue(Shooter.INSTANCE.shooterCloseShoot);
////                .whenBecomesFalse(new SequentialGroup(
////                        new Delay(0.75),
////                        Intake.INSTANCE.intakeFullSpeed,
////                        TransferPusher.INSTANCE.transferPush,
////                        new Delay(0.15),
////                        TransferPusher.INSTANCE.transferHold,
////                        new Delay(0.25),
////                        TransferPusher.INSTANCE.transferPush,
////                        new Delay(0.15),
////                        TransferPusher.INSTANCE.transferHold,
////                        new Delay(0.25),
////                        TransferPusher.INSTANCE.transferPush
////                ));
//        Gamepads.gamepad2().dpadDown()
//                .whenBecomesTrue(Shooter.INSTANCE.shooterOff);
//        Gamepads.gamepad2().y()
//                .whenBecomesTrue(TransferPusher.INSTANCE.transferPush)
//                .whenBecomesFalse(TransferPusher.INSTANCE.transferHold); //when button held transfer runs
//        Gamepads.gamepad2().a()
//                .whenBecomesTrue(TransferPusher.INSTANCE.transferReverse)
//                .whenBecomesFalse(TransferPusher.INSTANCE.transferHold); //when button held transfer reverses
//    }
//
//    @Override
//    public void onUpdate() { //runs every loop
//        BindingManager.update();
//        telemetry.addData("Robot Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
//        telemetry.addData("Robot x", PedroComponent.follower().getPose().getX());
//        telemetry.addData("Robot y", PedroComponent.follower().getPose().getY());
//        ActiveOpMode.telemetry().update();
//
//        if (Math.abs(Math.toDegrees(PedroComponent.follower().getPose().getHeading()) - 180) <= 2){ //if follower has heading of 180 degrees (with 2 degrees of tolerance), reset the IMU
//            new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180)))); //reset pinpoint IMU);
//        }
//
//    }
//
//    @Override
//    public void onStop() {
//        BindingManager.reset();
//        webcam.stop();
//    }
//}
