package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorLeft;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorRight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "6 Ball Red Far Auto")
public class Red6BallFarAuto extends NextFTCOpMode {

    public Red6BallFarAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    public Command shooterMotorsOn() {
        return new ParallelGroup(
                ShooterMotorLeft.INSTANCE.shooterMotorLeftOn(),
                ShooterMotorRight.INSTANCE.shooterMotorRightOn()
        );
    }

    public Command shooterMotorsOff() {
        return new ParallelGroup(
                ShooterMotorLeft.INSTANCE.shooterMotorLeftOff(),
                ShooterMotorRight.INSTANCE.shooterMotorRightOff()
        );
    }

    public Command shootWithTransfer() {
        return new SequentialGroup(
                shooterMotorsOn(),
                ShooterMotorRight.INSTANCE.waitUntilShooterRightAtTargetVelocity(125, targetVelocity, new SequentialGroup(
                        Intake.INSTANCE.intakeFullSpeed,
                        TransferPusher.INSTANCE.transferOn,
                        new Delay(0.15),
                        TransferPusher.INSTANCE.transferOff,
                        new Delay(0.25),
                        TransferPusher.INSTANCE.transferOn,
                        new Delay(0.15),
                        TransferPusher.INSTANCE.transferOff,
                        new Delay(0.25),
                        TransferPusher.INSTANCE.transferOn
                ))

        );
    }
    public Command autoRoutine() {
        return new SequentialGroup(
                new FollowPath(shoot1,true),
                shootWithTransfer(),
                new Delay(3),
                shooterMotorsOff(),
                TransferPusher.INSTANCE.transferOff,
                Intake.INSTANCE.intakeAutoSpeed,
                new FollowPath(turn1,true),
                new ParallelGroup(
                        new FollowPath(intake1,true),
                        createDistanceMarker(0.7, Intake.INSTANCE.intakeOneThirdSpeed)
                ),
                new Delay(1),
                Intake.INSTANCE.intakeOff,
                new FollowPath(shoot2,true),
                shootWithTransfer(),
                new Delay(3),
                shooterMotorsOff(),
                TransferPusher.INSTANCE.transferOff,
                new FollowPath(park,true)
        );
    }

    public static Command createDistanceMarker(double percentageOfPathTraveled, Command command) { //make sure to run this command parallel to followPath commands
        return new WaitUntil(() -> PedroComponent.follower().getPathCompletion() >= percentageOfPathTraveled)
                .then(command);
    }

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("Robot Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Robot x", PedroComponent.follower().getPose().getX());
        telemetry.addData("Robot y", PedroComponent.follower().getPose().getY());
        ActiveOpMode.telemetry().update();
    }

    private Path shoot1, turn1, intake1, shoot2, park;

    private final Pose startPose = new Pose(82.017, 7.096, Math.toRadians(270));
    private final Pose scoringPose = new Pose(86, 20, Math.toRadians(242));

    private final Pose turnPose = new Pose(97.461, 34.435, Math.toRadians(0));

    private final Pose intakePose1 = new Pose(133.687, 34.852, Math.toRadians(0));

    private final Pose endPose = new Pose(125, 34.852, Math.toRadians(0));


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());

        turn1 = new Path(new BezierLine(scoringPose, turnPose));
        turn1.setLinearHeadingInterpolation(scoringPose.getHeading(), turnPose.getHeading());

        intake1 = new Path(new BezierLine(turnPose, intakePose1));
        intake1.setLinearHeadingInterpolation(turnPose.getHeading(), intakePose1.getHeading());

        shoot2 = new Path(new BezierLine(intakePose1, scoringPose));
        shoot2.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

        park = new Path(new BezierLine(scoringPose, endPose));
        park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
    }

}
