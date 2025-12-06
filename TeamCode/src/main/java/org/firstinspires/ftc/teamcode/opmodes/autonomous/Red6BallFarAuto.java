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

@Configurable
@Autonomous(name = "6 Ball Red Far Auto")
public class Red6BallFarAuto extends NextFTCOpMode {

    // Timing constants
    public static double SHOOTER_SPINUP_TIME = 2;
    public static double BALL_TRANSFER_TIME = 1.0;
    public static double SHOT_PAUSE_TIME = 1.0;
    public static int SHOTS_PER_SEQUENCE = 3;

    // Velocity constraints
    public static double SLOW_VELOCITY = 1000;
    public static double MEDIUM_VELOCITY = 0.3;
    // Braking constants
    public static double BRAKING_STRENGTH = Constants.pathConstraints.getBrakingStrength();
    public static double BRAKING_START = Constants.pathConstraints.getBrakingStart();

    public New_Red9BallFarAuto() {
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
                new Delay(BALL_TRANSFER_TIME),
                TransferPusher.INSTANCE.transferHold,
                new Delay(SHOT_PAUSE_TIME)
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
                Intake.INSTANCE.intakeFullSpeed,
                shootWithTransfer(),
                new InstantCommand(() -> PedroComponent.follower().constants.useSecondaryHeadingPIDF = false),
                new FollowPath(intake1),
                Intake.INSTANCE.intakeFullSpeed,
                new FollowPath(intake1_slide),
                new FollowPath(intake_wiggle1),
                new FollowPath(intake2_slide),
                new FollowPath(intake_wiggle2),
                Intake.INSTANCE.intakeFullSpeed,
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

    private Path shoot1, shoot1Turn, shoot2, shoot2Turn, shoot3, park, intake1, intake1_slide, intake2_slide;
    private PathChain intake_wiggle1, intake_wiggle2, intake2;

    private final Pose startPose = new Pose(82, 9, Math.toRadians(270));

    private final Pose scoringPose = new Pose(85, 22, Math.toRadians(248));

    private final Pose turnPose1 = new Pose(117, 20, Math.toRadians(-90));

    private final Pose intake1ControlPose = new Pose(100.7, 6.5);
    private final Pose intake1ControlPose2 = new Pose(120, 13);

    private final Pose intakePose1 = new Pose(138, 26, Math.toRadians(-90));
    private final Pose intakePose_2 = new Pose(138, 15, Math.toRadians(-90));

    private final Pose intakeSlidePose = new Pose(138, 10, Math.toRadians(-90));

    private final Pose turnPose2 = new Pose(85, 36, Math.toRadians(0));

    private final Pose intakePose2 = new Pose(135, 36, Math.toRadians(0));

    private final Pose endPose = new Pose(108, 11, Math.toRadians(0));


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());
        shoot1.setHeadingConstraint(Math.toRadians(2));
        shoot1.setTimeoutConstraint(500);


        shoot1Turn = new Path(new BezierPoint(scoringPose));
        shoot1Turn.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose1.getHeading());

        intake1 = new Path(new BezierCurve(scoringPose, intake1ControlPose, intake1ControlPose2, intakePose1));
//        intake1.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose1.getHeading());
        intake1.setHeadingInterpolation(HeadingInterpolator.piecewise(
                new HeadingInterpolator.PiecewiseNode(0, 0.8, HeadingInterpolator.constant(Math.toRadians(-60))),
                new HeadingInterpolator.PiecewiseNode(0.8, 1, HeadingInterpolator.constant(Math.toRadians(-90)))
        ));
//        HeadingInterpolator headingInterpolator = HeadingInterpolator.
//        intake1.setBrakingStart(BRAKING_START);
//        intake1.setBrakingStrength(BRAKING_STRENGTH);
//        intake1.setVelocityConstraint(SLOW_VELOCITY);



        intake1_slide = new Path(new BezierLine(intakePose1, intakePose_2));
        intake1_slide.setLinearHeadingInterpolation(intakePose1.getHeading(), intakePose_2.getHeading());
        intake1_slide.setVelocityConstraint(SLOW_VELOCITY);
//        intake1_slide.setTranslationalConstraint(8);

        intake_wiggle1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierPoint(intakePose_2))
                .setLinearHeadingInterpolation(intakePose_2.getHeading(), intakePose_2.getHeading() + Math.toRadians(10))
                .addPath(new BezierPoint(intakePose_2))
                .setLinearHeadingInterpolation(intakePose_2.getHeading(), intakePose_2.getHeading() + Math.toRadians(-15))
                .build();

        intake2_slide = new Path(new BezierLine(intakePose_2, intakeSlidePose));
        intake2_slide.setLinearHeadingInterpolation(intakePose_2.getHeading(), intakeSlidePose.getHeading());
        intake2_slide.setVelocityConstraint(SLOW_VELOCITY);
        intake2_slide.setTranslationalConstraint(8);

        intake_wiggle2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierPoint(intakeSlidePose))
                .setLinearHeadingInterpolation(intakeSlidePose.getHeading(), intakeSlidePose.getHeading() + Math.toRadians(10))
                .addPath(new BezierPoint(intakeSlidePose))
                .setLinearHeadingInterpolation(intakeSlidePose.getHeading(), intakeSlidePose.getHeading() + Math.toRadians(-15))
                .build();

        shoot2 = new Path(new BezierLine(intakeSlidePose, scoringPose));
        shoot2.setConstantHeadingInterpolation(intakeSlidePose.getHeading());

        shoot2Turn = new Path(new BezierPoint(scoringPose));
        shoot2Turn.setLinearHeadingInterpolation(intakeSlidePose.getHeading(), scoringPose.getHeading() + Math.toRadians(4));
        shoot2Turn.setHeadingConstraint(Math.toRadians(2));
        shoot2Turn.setTimeoutConstraint(500);

        intake2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoringPose, turnPose2))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), turnPose2.getHeading())
                .addPath(new BezierLine(turnPose2, intakePose_2))
                .setLinearHeadingInterpolation(turnPose2.getHeading(), intakePose_2.getHeading())
                .build();

        shoot3 = new Path(new BezierLine(intakePose_2, scoringPose));
        shoot3.setLinearHeadingInterpolation(intakePose_2.getHeading(), scoringPose.getHeading());

        park = new Path(new BezierLine(scoringPose, endPose));
        park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
    }

}
