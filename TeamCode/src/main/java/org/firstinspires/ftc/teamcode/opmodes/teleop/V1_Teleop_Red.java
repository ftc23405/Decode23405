package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
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

@TeleOp(name = "V1 RED Side Teleop")
@Configurable //panels
public class V1_Teleop_Red extends NextFTCOpMode {

    public V1_Teleop_Red() {
        addComponents( //add needed components
                new SubsystemComponent(Intake.INSTANCE),
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
        PedroComponent.follower().update();

        parkPath = PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, parkingPose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(PedroComponent.follower()::getHeading, parkingPose.getHeading(), 0.8))
                .build();
    }

    @Override
    public void onStartButtonPressed() {

        PedroDriverControlled driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().start()
                .whenBecomesTrue(new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose())));
        Gamepads.gamepad1().a()
                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed);
        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Shooter.INSTANCE.shooterOn);
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Shooter.INSTANCE.shooterOff);
        parkButton
                .whenBecomesTrue(new FollowPath(parkPath));
    }

    @Override
    public void onUpdate() {
        ActiveOpMode.telemetry();
        BindingManager.update();
        PedroComponent.follower().update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
