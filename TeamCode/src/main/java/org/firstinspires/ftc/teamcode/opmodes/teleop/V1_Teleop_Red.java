package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

@TeleOp(name = "V1 RED Side Teleop")
@Configurable //panels
public class V1_Teleop_Red extends NextFTCOpMode {

    public V1_Teleop_Red() {
        addComponents( //add needed components
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    //not in use (using pedro teleop), create motors and set directions
    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor");
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").reversed();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor");
    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();

    private final Pose parkingPose = new Pose(38.7,33.2, Math.toRadians(90));

    private PathChain parkPath;

    Button parkButton = (Gamepads.gamepad1().dpadUp()).and(Gamepads.gamepad2().dpadUp());


    @Override
    public void onInit() {

        PedroComponent.follower().setStartingPose(new Pose(0,0, Math.toRadians(180)));

        parkPath = PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, parkingPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, parkingPose.getHeading(), 0.8))
                .build();
    }

    @Override
    public void onStartButtonPressed() {

        PedroDriverControlled driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().start()
                .whenBecomesTrue(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180))));
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed);
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().b()
                .whenBecomesTrue(Intake.INSTANCE.intakeReverseHalfSpeed);


        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(Shooter.INSTANCE.shooterOn);
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(Shooter.INSTANCE.shooterOff);
        Gamepads.gamepad2().y()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferOn)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff);
        Gamepads.gamepad2().a()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferReverse)
                .whenBecomesFalse(TransferPusher.INSTANCE.transferOff);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(TransferPusher.INSTANCE.transferOff);
        parkButton
                .whenBecomesTrue(new FollowPath(parkPath));
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Robot Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Robot x", PedroComponent.follower().getPose().getX());
        telemetry.addData("Robot y", PedroComponent.follower().getPose().getY());

    }

    @Override
    public void onStop() {
    }
}
