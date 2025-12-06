package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
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
@Autonomous(name = "New 9 Ball Red Classifier Auto")
public class New_Red9BallClassifierAuto extends NextFTCOpMode {


    static double SHOOTER_SPINUP_TIME = 1.5;
    static double BALL_TRANSFER_TIME = 1.0;
    static double SHOT_PAUSE_TIME = 1.0;
    static int SHOTS_PER_SEQUENCE = 3;

    // Velocity constraints
    static double SLOW_VELOCITY = 10;
    static double MEDIUM_VELOCITY = 0.3;
    // Braking constants
    static double BRAKING_STRENGTH = Constants.pathConstraints.getBrakingStrength();
    static double BRAKING_START = Constants.pathConstraints.getBrakingStart();

    public New_Red9BallClassifierAuto() {
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

    private final Pose startPose = new Pose(111, 135, Math.toRadians(180));

    private final Pose scoringPose = new Pose(92, 94, Math.toRadians(230));

    private final Pose turnPose1 = new Pose(92, 84, Math.toRadians(0));

    private final Pose intakePose1 = new Pose(129, 84, Math.toRadians(0));

    private final Pose turnPose2 = new Pose(92, 59, Math.toRadians(0));

    private final Pose intakePose2 = new Pose(135, 59, Math.toRadians(0));

    private final Pose goBack2ControlPose = new Pose(88,67);

    private final Pose endPose = new Pose(129, 94, Math.toRadians(0));


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());

        intake1 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoringPose, turnPose1))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), turnPose1.getHeading())
                .addPath(new BezierLine(turnPose1, intakePose1))
                .setConstantHeadingInterpolation(turnPose1.getHeading())
                .build();

        shoot2 = new Path(new BezierLine(intakePose1, scoringPose));
        shoot2.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

        intake2 = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scoringPose, turnPose2))
                .setLinearHeadingInterpolation(scoringPose.getHeading(), turnPose2.getHeading())
                .addPath(new BezierLine(turnPose2, intakePose2))
                .setConstantHeadingInterpolation(turnPose2.getHeading())
                .build();

        shoot3 = new Path(new BezierCurve(intakePose2, goBack2ControlPose, scoringPose));
        shoot3.setLinearHeadingInterpolation(intakePose2.getHeading(), scoringPose.getHeading());

        park = new Path(new BezierLine(scoringPose, endPose));
        park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
    }

}
