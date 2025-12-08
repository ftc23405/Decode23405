package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorLeft;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorRight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.pedroPathing.WiggleInterpolator;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

@Configurable
@Autonomous(name = "Simple 6 Ball Blue Far Auto")
public class SimpleBlue6BallFarAuto extends NextFTCOpMode {

    // Timing constants
    // Braking constants
    public static double BRAKING_STRENGTH = Constants.pathConstraints.getBrakingStrength();
    public static double BRAKING_START = Constants.pathConstraints.getBrakingStart();

    public SimpleBlue6BallFarAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,
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

    /**
     * Creates a single shot sequence: push transfer, wait for ball, hold transfer, pause
     */
    private Command createShotSequence() {
        return new SequentialGroup(
                new ParallelGroup(Intake.INSTANCE.intakeAutoSpeed, TransferPusher.INSTANCE.transferPush),
                new ParallelGroup(new SequentialGroup(
                        Intake.INSTANCE.intakeReverseHalfSpeed,
                        new Delay(0.2), 
                        Intake.INSTANCE.intakeAutoSpeed
                ), new SequentialGroup(
                        new Delay(BALL_TRANSFER_TIME),
                        TransferPusher.INSTANCE.transferHold,
                        new Delay(SHOT_PAUSE_TIME)
                ))
        );
    }

    /**
     * Executes a complete shooting sequence with multiple balls
     */
    public Command shootWithTransfer() {
        return new SequentialGroup(
                TransferPusher.INSTANCE.transferHold,
                shooterMotorsOn(),
                new Delay(SHOOTER_SPINUP_TIME),
                createShotSequence(),
                createShotSequence(),
                createShotSequence(),
                Intake.INSTANCE.intakeOff,
                shooterMotorsOff()
        );
    }
    public Command autoRoutine() {
        return new SequentialGroup(
                TransferPusher.INSTANCE.transferHold,
                new FollowPath(shoot1),
                shootWithTransfer(),
                new InstantCommand(() -> PedroComponent.follower().constants.useSecondaryHeadingPIDF = false),
                new FollowPath(intake1),
                Intake.INSTANCE.intakeFullSpeed,
                new FollowPath(intake2),
                Intake.INSTANCE.intakeOff,
                new FollowPath(shoot2),
                new InstantCommand(() -> PedroComponent.follower().constants.useSecondaryHeadingPIDF = true),
                new FollowPath(shoot2Turn),
                new InstantCommand(() -> PedroComponent.follower().constants.useSecondaryHeadingPIDF = false),
                shootWithTransfer(),
//                Intake.INSTANCE.intakeFullSpeed,
//                new FollowPath(intake2),
                Intake.INSTANCE.intakeOff,
//                new InstantCommand(() -> PedroComponent.follower().constants.useSecondaryHeadingPIDF = true),
//                new FollowPath(shoot3),
//                new InstantCommand(() -> PedroComponent.follower().constants.useSecondaryHeadingPIDF = false),
//                shootWithTransfer(),
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

    private Path shoot1, shoot2, shoot2Turn, shoot3, park, intake1, intake2;

    private final Pose startPose = new Pose(82, 9, Math.toRadians(270)).mirror();

    private final Pose scoringPose = new Pose(85, 22, Math.toRadians(248)).mirror();

    private final Pose intake1Pose = new Pose(95, 33.5, Math.toRadians(0)).mirror();

    private final Pose intake2Pose = new Pose(136, 33.5, Math.toRadians(0)).mirror();


    private final Pose endPose = new Pose(108, 11, Math.toRadians(0)).mirror();


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());
        shoot1.setHeadingConstraint(Math.toRadians(2));
        shoot1.setTimeoutConstraint(500);

        intake1 = new Path(new BezierLine(scoringPose, intake1Pose));
        intake1.setLinearHeadingInterpolation(scoringPose.getHeading(), intake1Pose.getHeading());

        intake2 = new Path(new BezierLine(intake1Pose, intake2Pose));
        intake2.setConstantHeadingInterpolation(intake1Pose.getHeading());
        intake2.setVelocityConstraint(10);

        shoot2 = new Path(new BezierLine(intake2Pose, scoringPose));
        shoot2.setLinearHeadingInterpolation(intake2Pose.getHeading(), scoringPose.getHeading());

        shoot2Turn = new Path(new BezierPoint(scoringPose));
        shoot2Turn.setConstantHeadingInterpolation(scoringPose.getHeading());
        shoot2Turn.setHeadingConstraint(Math.toRadians(2));
        shoot2Turn.setTimeoutConstraint(500);

        park = new Path(new BezierLine(scoringPose, endPose));
        park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
    }

}
