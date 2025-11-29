package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
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
@Autonomous(name = "Complementary 9 Ball Blue Side Far Auto")
public class Blue9BallFarStartComplementAuto extends NextFTCOpMode{

    public Blue9BallFarStartComplementAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    public Command shooterMotorsAutoClassifierShot() {
        return new ParallelGroup(
                ShooterMotorLeft.INSTANCE.shooterMotorAutoLeftClassifier(),
                ShooterMotorRight.INSTANCE.shooterMotorAutoRightClassifier()
        );
    }

    public Command shooterMotorsOff() {
        return new ParallelGroup(
                ShooterMotorLeft.INSTANCE.shooterMotorLeftOff(),
                ShooterMotorRight.INSTANCE.shooterMotorRightOff()
        );
    }

    public Command shooterMotorsReverse() {
        return new SequentialGroup(
                ShooterMotorLeft.INSTANCE.shooterMotorLeftReverse(),
                ShooterMotorRight.INSTANCE.shooterMotorRightReverse()
        );
    }


    public Command shootWithTransfer() {
        return new SequentialGroup(
                shooterMotorsAutoClassifierShot(),
                new Delay(0.5),
                Intake.INSTANCE.intakeFullSpeed,
                TransferPusher.INSTANCE.transferPush,
                new Delay(0.15),
                TransferPusher.INSTANCE.transferHold,
                new Delay(0.25),
                TransferPusher.INSTANCE.transferPush,
                new Delay(0.15),
                TransferPusher.INSTANCE.transferHold,
                new Delay(0.25),
                TransferPusher.INSTANCE.transferPush
        );
    }

    public Command autoRoutine() {
        return new SequentialGroup(
                new FollowPath(shoot1,true),
                shootWithTransfer(),
                new Delay(1),
                shooterMotorsOff(),
                TransferPusher.INSTANCE.transferHold,
                Intake.INSTANCE.intakeAutoSpeed,
                new ParallelGroup(
                        new FollowPath(intake1,true),
                        createDistanceMarker(0.9, Intake.INSTANCE.intakeOneThirdSpeed)
                ),
                new Delay(1),
                Intake.INSTANCE.intakeOff,
                new FollowPath(goBack1,true),
                shootWithTransfer(),
                new Delay(2),
                shooterMotorsOff(),
                TransferPusher.INSTANCE.transferHold,
                Intake.INSTANCE.intakeAutoSpeed,
                new ParallelGroup(
                        new FollowPath(intake2,true),
                        createDistanceMarker(0.9, Intake.INSTANCE.intakeOneThirdSpeed)
                ),
                new Delay(1),
                Intake.INSTANCE.intakeOff,
                new FollowPath(goBack2,true),
                shootWithTransfer(),
                new Delay(1),
                shooterMotorsOff(),
                TransferPusher.INSTANCE.transferHold,
                Intake.INSTANCE.intakeOff,
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

    private Path shoot1, intake1, goBack1, intake2, goBack2, park;

    private final Pose startPose = new Pose(82.017, 7.096, Math.toRadians(270)).mirror();
    private final Pose scoringPose = new Pose(84, 100, Math.toRadians(217)).mirror();

    private final Pose scoringPoseOffset = new Pose(86, 22, Math.toRadians(220)).mirror();

    private final Pose intakePose1 = new Pose(132.150, 86.990, Math.toRadians(0)).mirror();
    private final Pose intakeShortControlPose1 = new Pose(75.982, 91.224).mirror();
    private final Pose intakeShortControlPose2 = new Pose(70.992, 85.990).mirror();

    private final Pose intakePose2 = new Pose(137.298, 64.523, Math.toRadians(0)).mirror();
    private final Pose intakeLongControlPose1 = new Pose(65.879, 53.733).mirror();
    private final Pose intakeLongControlPose2 = new Pose(94.241, 61.158).mirror();

    private final Pose goBackLongControlPose1 = new Pose(81.825, 72.965).mirror();

    private final Pose endPose = new Pose(129, 87, Math.toRadians(0)).mirror();


    public void buildPaths() {
        shoot1 = new Path(new BezierLine(startPose, scoringPose));
        shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading(), 0.8);

        intake1 = new Path(new BezierCurve(scoringPose, intakeShortControlPose1,intakeShortControlPose2, intakePose1));
        intake1.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose1.getHeading(), 0.5);

        goBack1 = new Path(new BezierLine(intakePose1, scoringPose));
        goBack1.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

        intake2 = new Path(new BezierCurve(scoringPose, intakeLongControlPose1, intakeLongControlPose2, intakePose2));
        intake2.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose2.getHeading(), 0.5);

        goBack2 = new Path(new BezierCurve(intakePose2, goBackLongControlPose1, scoringPose));
        goBack2.setLinearHeadingInterpolation(intakePose2.getHeading(), scoringPoseOffset.getHeading());

        park = new Path(new BezierLine(scoringPose, endPose));
        park.setLinearHeadingInterpolation(scoringPoseOffset.getHeading(), endPose.getHeading());
    }

}
