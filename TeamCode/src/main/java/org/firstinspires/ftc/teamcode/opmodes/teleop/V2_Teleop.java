package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.commandbase.vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
@Configurable //panels
public class V2_Teleop extends NextFTCOpMode {

    public V2_Teleop() {
        addComponents( //add needed components
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower) //follower for Pedro teleop drive
        );
    }
    private final Pose parkingPose = new Pose(38.7,33.2, Math.toRadians(90));

    private TelemetryManager telemetryM;

    private PathChain parkPath;

    private AprilTagWebcam webcam;

    Button parkButton = (Gamepads.gamepad1().dpadUp()).and(Gamepads.gamepad2().dpadUp());


    @Override
    public void onInit() {

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
                .whenBecomesTrue(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(0)))); //reset pinpoint IMU

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new SequentialGroup(
                        Shooter.INSTANCE.shooterOn,
                        new Delay(0.75),
                        Intake.INSTANCE.intakeFullSpeed,
                        TransferPusher.INSTANCE.transferOn
                ))
                .whenBecomesFalse(Intake.INSTANCE.intakeOff)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff);
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Shooter.INSTANCE.shooterOff);
        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.intakeHalfSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Intake.INSTANCE.intakeQuarterSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().b()
                .whenBecomesTrue(Intake.INSTANCE.intakeReverseFullSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);


        Gamepads.gamepad2().y()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferOn)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff); //when button held transfer runs
        Gamepads.gamepad2().a()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferReverse)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff); //when button held transfer reverses
    }

    @Override
    public void onUpdate() { //runs every loop
        BindingManager.update();
        telemetry.addData("Robot Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Robot x", PedroComponent.follower().getPose().getX());
        telemetry.addData("Robot y", PedroComponent.follower().getPose().getY());
        ActiveOpMode.telemetry().update();


    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
