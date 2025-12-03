package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorLeft;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorRight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;

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
@Autonomous(name = "New 9 Ball Red Far Auto")
public class New_Red9BallFarAuto extends NextFTCOpMode {

    public New_Red9BallFarAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    public Command shooterMotorsOn() {
        return new ParallelGroup(
                ShooterMotorLeft.INSTANCE.shooterMotorLeftFar(),
                ShooterMotorRight.INSTANCE.shooterMotorRightFar()
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
                new Delay(2),
                new ParallelGroup(Intake.INSTANCE.intakeAutoSpeed, TransferPusher.INSTANCE.transferPush),
                new Delay(0.2),
                Intake.INSTANCE.intakeOff,
                TransferPusher.INSTANCE.transferHold,
                new ParallelGroup(Intake.INSTANCE.intakeAutoSpeed, TransferPusher.INSTANCE.transferPush),
                new Delay(0.2),
                Intake.INSTANCE.intakeOff,
                TransferPusher.INSTANCE.transferHold,
                new ParallelGroup(Intake.INSTANCE.intakeAutoSpeed, TransferPusher.INSTANCE.transferPush),
                new Delay(0.2),
                Intake.INSTANCE.intakeOff,
                TransferPusher.INSTANCE.transferHold,
                shooterMotorsOff()
        );
    }
    public Command autoRoutine() {
        return new SequentialGroup(
                new FollowPath(shoot1),
                shootWithTransfer(),
                Intake.INSTANCE.intakeFullSpeed,
                new FollowPath(intake1),
                Intake.INSTANCE.intakeOff,
                new FollowPath(shoot2),
                shootWithTransfer(),
                Intake.INSTANCE.intakeFullSpeed,
                new FollowPath(intake2),
                Intake.INSTANCE.intakeOff,
                new FollowPath(shoot3),
                shootWithTransfer(),
                new FollowPath(park)
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
        Drawing.drawPoseHistory(PedroComponent.follower().getPoseHistory());
        Drawing.drawDebug(PedroComponent.follower());
        telemetry.addData("Robot Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        telemetry.addData("Robot x", PedroComponent.follower().getPose().getX());
        telemetry.addData("Robot y", PedroComponent.follower().getPose().getY());
        ActiveOpMode.telemetry().update();
    }

    private Path shoot1, shoot2, shoot3, park;
    private PathChain intake1, intake2;

    private final Pose startPose = new Pose(82, 9, Math.toRadians(270));

    private final Pose scoringPose = new Pose(85, 22, Math.toRadians(250));

    private final Pose turnPose1 = new Pose(117, 18, Math.toRadians(0));

    private final Pose intakePose1 = new Pose(132, 18, Math.toRadians(-20));

    private final Pose intakeSlidePose = new Pose(134, 8, Math.toRadians(-40));

    private final Pose turnPose2 = new Pose(85, 36, Math.toRadians(0));

    private final Pose intakePose2 = new Pose(135, 36, Math.toRadians(0));

    private final Pose endPose = new Pose(108, 11, Math.toRadians(0));


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());


        intake1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoringPose, turnPose1))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), turnPose1.getHeading())
                .addPath(new BezierLine(turnPose1, intakePose1))
                .setLinearHeadingInterpolation(turnPose1.getHeading(), intakePose1.getHeading())
                .addPath(new BezierLine(intakePose1, intakeSlidePose))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), intakeSlidePose.getHeading())
                .build();

        shoot2 = new Path(new BezierLine(intakePose1, scoringPose));
        shoot2.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

        intake2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoringPose, turnPose2))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), turnPose2.getHeading())
                .addPath(new BezierLine(turnPose2, intakePose2))
                .setLinearHeadingInterpolation(turnPose2.getHeading(), intakePose2.getHeading())
                .build();

        shoot3 = new Path(new BezierLine(intakePose2, scoringPose));
        shoot3.setLinearHeadingInterpolation(intakePose2.getHeading(), scoringPose.getHeading());

        park = new Path(new BezierLine(scoringPose, endPose));
        park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
    }

}
