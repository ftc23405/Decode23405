package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorLeft;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorRight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.PerpetualCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

@TeleOp
@Configurable //panels
public class V3_Teleop extends NextFTCOpMode {

    public V3_Teleop() {
        addComponents( //add needed components
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new PedroComponent(Constants::createFollower) //follower for Pedro teleop drive
        );
    }
    private final Pose parkingPose = new Pose(38.7,33.2, Math.toRadians(90));

    private TelemetryManager telemetryM;

    private Limelight3A limelight;

    private TurnBy currentTurn;

    private PathChain parkPath;

//    AprilTagWebcam webcam = new AprilTagWebcam();

    Button parkButton = (Gamepads.gamepad1().dpadUp()).and(Gamepads.gamepad2().dpadUp());


    @Override
    public void onInit() {
//        webcam.initalize(hardwareMap, telemetryM);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        PedroComponent.follower().setStartingPose(new Pose(0,0, Math.toRadians(180))); //set starting pose for pinpoint IMU

    }

    @Override
    public void onStartButtonPressed() {

        PedroDriverControlled driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate(), //negate the right stick because it's inverted
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().start()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180)))),
                        new InstantCommand(() -> gamepad1.rumble(500))
                )); //reset pinpoint IMU

//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(() -> webcam.start())
//                .whenTrue(new SequentialGroup(
//                        new InstantCommand(() -> webcam.update()),
//                        new TurnBy(Angle.fromDeg(webcam.getTagBearing(24)))) //get red tag bearing
//                )
//                .whenFalse(() -> webcam.pause()); // stop streaming to save CPU

//        Gamepads.gamepad1().y()
//                .whenBecomesTrue(() -> {
//                    limelight.start();
//                    LLResult llResult = limelight.getLatestResult();
//                    if (llResult != null && llResult.isValid()) {
//                        currentTurn = new TurnBy(Angle.fromDeg(-llResult.getTx()));
//                        currentTurn.setInterruptible(true);
//                        currentTurn.schedule();
//                        telemetry.addData("Turning by", -llResult.getTx());
//                    } else {
//                        telemetry.addLine("No valid tag detected!");
//                    }
//                    telemetry.update();
//                })
//                .whenBecomesFalse(() -> {
//
//                    if (currentTurn != null) {
//                        CommandManager.INSTANCE.cancelCommand(currentTurn);
//                        limelight.stop();
//                        driverControlled.schedule();
//                    }
//                    else {
//                        driverControlled.schedule();
//                    }
//                });


        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new ParallelGroup(
                        ShooterMotorLeft.INSTANCE.shooterMotorLeftClassifier(),
                        ShooterMotorRight.INSTANCE.shooterMotorRightClassifier()
                ));
        Gamepads.gamepad1().dpadUp()
                        .whenBecomesTrue(new ParallelGroup(
                                ShooterMotorLeft.INSTANCE.shooterMotorLeftOn(),
                                ShooterMotorRight.INSTANCE.shooterMotorRightOn()
                        ));
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(new ParallelGroup(
                        ShooterMotorLeft.INSTANCE.shooterMotorLeftOff(),
                        ShooterMotorRight.INSTANCE.shooterMotorRightOff()
                ));

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferOn)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff); //when button held transfer runs
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferReverse)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff); //when button held transfer reverses

        Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferSlowReverse)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff); //when button held transfer slow reverses


        Gamepads.gamepad2().y()
                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(Intake.INSTANCE.intakeHalfSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad2().a()
                .whenBecomesTrue(Intake.INSTANCE.intakeOneThirdSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad2().b()
                .whenBecomesTrue(Intake.INSTANCE.intakeReverseFullSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
    }

    @Override
    public void onUpdate() { //runs every loop
        BindingManager.update();
        telemetry.addData("Robot Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Robot x", PedroComponent.follower().getPose().getX());
        telemetry.addData("Robot y", PedroComponent.follower().getPose().getY());
        ActiveOpMode.telemetry().update();

        if (Math.abs(Math.toDegrees(PedroComponent.follower().getPose().getHeading()) - 180) <= 2){ //if follower has heading of 180 degrees (with 2 degrees of tolerance), reset the IMU
            new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180)))); //reset pinpoint IMU);
        }

//        if (ShooterMotorRight.INSTANCE.isAtTarget(classifierVelocity, 150)) {
//            gamepad1.rumble(200);
//        }

    }

    @Override
    public void onStop() {
        BindingManager.reset();
        limelight.stop();
    }
}
