package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commandbase.vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
@Configurable //panels
public class TestTeleop extends NextFTCOpMode {

    public TestTeleop() {
        addComponents( //add needed components
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new PedroComponent(Constants::createFollower) //follower for Pedro teleop drive
        );
    }
    private final Pose parkingPose = new Pose(38.7,33.2, Math.toRadians(90));

    private TelemetryManager telemetryM;

    private Limelight3A limelight;

    private PathChain parkPath;


    TurnBy currentTurn;

    Button parkButton = (Gamepads.gamepad1().dpadUp()).and(Gamepads.gamepad2().dpadUp());



    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        PedroComponent.follower().setStartingPose(new Pose(0,0, Math.toRadians(180))); //set starting pose for pinpoint IMU

    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();

        PedroDriverControlled driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate(), //negate the right stick because it's inverted
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180)))),
                        new InstantCommand(() -> gamepad1.rumble(500))
                )); //reset pinpoint IMU

        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    LLResult llResult = limelight.getLatestResult();
                    if (llResult != null && llResult.isValid()) {
                        currentTurn = new TurnBy(Angle.fromDeg(-llResult.getTx()));
                        currentTurn.schedule();
                        telemetry.addData("Turning by", llResult.getTx());
                    } else {
                        telemetry.addLine("No valid tag detected!");
                    }
                    telemetry.update();
                })
                .whenBecomesFalse(() -> {
                    CommandManager.INSTANCE.cancelAll();
                    driverControlled.schedule();
        });
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

    }

    @Override
    public void onStop() {
        BindingManager.reset();
        limelight.stop();
    }
}