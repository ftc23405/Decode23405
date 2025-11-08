package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "9 Ball Red Side Classifier Auto")
public class Red9BallClassifierAuto extends NextFTCOpMode{

        public Red9BallClassifierAuto() {
            addComponents(
                    new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                    new SubsystemComponent(TransferPusher.INSTANCE),
                    BulkReadComponent.INSTANCE,
                    new PedroComponent(Constants::createFollower)
            );
        }

    public Command shootWithTransfer() {
        return new SequentialGroup(
                Shooter.INSTANCE.shooterClassifierShoot,
                new Delay(1),
                Intake.INSTANCE.intakeFullSpeed,
                TransferPusher.INSTANCE.transferOn,
                new Delay(0.2),
                TransferPusher.INSTANCE.transferOff,
                new Delay(0.3),
                TransferPusher.INSTANCE.transferOn,
                new Delay(0.2),
                TransferPusher.INSTANCE.transferOff,
                new Delay(0.3),
                TransferPusher.INSTANCE.transferOn
        );
    }

        public Command autoRoutine() {
            return new SequentialGroup(
                    new FollowPath(shoot1,true),
                    shootWithTransfer(),
                    new Delay(3),
                    Shooter.INSTANCE.shooterOff,
                    TransferPusher.INSTANCE.transferOff,
                    Intake.INSTANCE.intakeHalfSpeed,
                    new FollowPath(intake1,true),
                    new Delay(1),
                    Intake.INSTANCE.intakeOff,
                    new FollowPath(goBack1,true),
                    shootWithTransfer(),
                    new Delay(3),
                    Shooter.INSTANCE.shooterOff,
                    TransferPusher.INSTANCE.transferOff,
                    new FollowPath(intake2,true)
//                    new Delay(1),
//                    Intake.INSTANCE.intakeOff,
//                    new FollowPath(goBack2,true),
//                    shootWithTransfer(),
//                    new Delay(3),
//                    Shooter.INSTANCE.shooterOff,
//                    TransferPusher.INSTANCE.transferOff,
//                    Intake.INSTANCE.intakeOff,
//                    new FollowPath(park,true)

            );
        }

        public static Command createDistanceMarker(double distance, Command command) { //make sure to run this command parallel to followPath commands
            return new WaitUntil(() -> PedroComponent.follower().getDistanceTraveledOnPath() >= distance)
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

        private Path shoot1, intake1, goBack1, intake2, goBack2, park;

        private final Pose startPose = new Pose(120, 132, Math.toRadians(216));
        private final Pose scoringPose = new Pose(84, 100, Math.toRadians(216));

        private final Pose intakePose1 = new Pose(130.150, 84.990, Math.toRadians(0));
        private final Pose intakeShortControlPose1 = new Pose(75.982, 90.224);
        private final Pose intakeShortControlPose2 = new Pose(70.992, 84.990);

        private final Pose intakePose2 = new Pose(129.298, 60.523, Math.toRadians(0));
        private final Pose intakeLongControlPose1 = new Pose(65.879, 52.733);
        private final Pose intakeLongControlPose2 = new Pose(94.241, 60.158);

        private final Pose goBackLongControlPose1 = new Pose(81.825, 71.965);

        private final Pose endPose = new Pose(125.620, 70.600, Math.toRadians(180));


        public void buildPaths() {
            shoot1 = new Path(new BezierLine(startPose, scoringPose));
            shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading(), 0.8);

            intake1 = new Path(new BezierCurve(scoringPose, intakeShortControlPose1,intakeShortControlPose2, intakePose1));
            intake1.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose1.getHeading(), 0.5);

            goBack1 = new Path(new BezierLine(intakePose1, scoringPose));
            goBack1.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

            intake2 = new Path(new BezierCurve(scoringPose, intakeLongControlPose1, intakeLongControlPose2, intakePose2));
            intake2.setLinearHeadingInterpolation(scoringPose.getHeading(), intakePose2.getHeading(), 0.6);

            goBack2 = new Path(new BezierCurve(intakePose2, goBackLongControlPose1, scoringPose));
            goBack2.setLinearHeadingInterpolation(intakePose2.getHeading(), scoringPose.getHeading());

            park = new Path(new BezierLine(scoringPose, endPose));
            park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
        }

}
