package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
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
import dev.nextftc.core.commands.CommandManager;
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

    // Timing constants
    public static double SHOOTER_SPINUP_TIME = 1.5;
    public static double BALL_TRANSFER_TIME = 1.0;
    public static double SHOT_PAUSE_TIME = 1.0;
    public static int SHOTS_PER_SEQUENCE = 3;

    // Velocity constraints
    public static double SLOW_VELOCITY = 10;
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
                Intake.INSTANCE.intakeAutoSpeed,
                //shootWithTransfer(),
                new FollowPath(intake1),
                Intake.INSTANCE.intakeFullSpeed,
                new FollowPath(intake1_slide),
                new FollowPath(intake_wiggle),
                Intake.INSTANCE.intakeOff,
                new FollowPath(shoot2),
                shootWithTransfer(),
//                Intake.INSTANCE.intakeFullSpeed,
//                new FollowPath(intake2),
                Intake.INSTANCE.intakeOff,
//                new FollowPath(shoot3),
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

    private Path shoot1, shoot2, shoot3, park, intake1, intake1_slide;
    private PathChain intake_wiggle, intake2;

    private final Pose startPose = new Pose(82, 9, Math.toRadians(270));

    private final Pose scoringPose = new Pose(85, 22, Math.toRadians(248));

    private final Pose turnPose1 = new Pose(117, 20, Math.toRadians(-90));

    private final Pose intake1ControlPose = new Pose(100.7, 6.5);

    private final Pose intakePose1 = new Pose(135, 25, Math.toRadians(-90));

    private final Pose intakeSlidePose = new Pose(139, 10, Math.toRadians(-90));

    private final Pose turnPose2 = new Pose(85, 36, Math.toRadians(0));

    private final Pose intakePose2 = new Pose(135, 36, Math.toRadians(0));

    private final Pose endPose = new Pose(108, 11, Math.toRadians(0));


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());
        shoot1.setHeadingConstraint(Math.toRadians(2));
        shoot1.setTimeoutConstraint(500);

        intake1 = new Path(new BezierCurve(scoringPose, intake1ControlPose, intakePose1));
        intake1.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose1.getHeading());
        intake1.setBrakingStart(BRAKING_START);
        intake1.setBrakingStrength(BRAKING_STRENGTH);
        intake1.setVelocityConstraint(SLOW_VELOCITY);

        intake1_slide = new Path(new BezierLine(intakePose1, intakeSlidePose));
        intake1_slide.setLinearHeadingInterpolation(intakePose1.getHeading(), intakeSlidePose.getHeading());
        intake1_slide.setVelocityConstraint(SLOW_VELOCITY);
        intake1_slide.setTranslationalConstraint(8);

        intake_wiggle = PedroComponent.follower().pathBuilder()
                .addPath(new BezierPoint(intakeSlidePose))
                .setLinearHeadingInterpolation(intakeSlidePose.getHeading(), intakeSlidePose.getHeading() + Math.toRadians(10))
                .addPath(new BezierPoint(intakeSlidePose))
                .setLinearHeadingInterpolation(intakeSlidePose.getHeading(), intakeSlidePose.getHeading() + Math.toRadians(-10))
                .build();

        shoot2 = new Path(new BezierLine(intakeSlidePose, scoringPose));
        shoot2.setLinearHeadingInterpolation(intakeSlidePose.getHeading(), scoringPose.getHeading());

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
