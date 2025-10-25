package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "9 Ball Red Side Classifier Auto")
public class Red9BallClassifierAuto extends NextFTCOpMode{

        public Red9BallClassifierAuto() {
            addComponents(
                    BulkReadComponent.INSTANCE,
                    new PedroComponent(Constants::createFollower)
            );
        }
        public Command autoRoutine() {
            return new SequentialGroup(
                    new FollowPath(shoot1,true),
                    new FollowPath(intake1,true),
                    new FollowPath(goBack1,true),
                    new FollowPath(intake2,true),
                    new FollowPath(goBack2,true),
                    new FollowPath(park,true)

            );
        }

        public static Command createDistanceMarker(double distance, Command command) { //make sure to run this command parallel to followPath commands
            return new WaitUntil(() -> PedroComponent.follower().getDistanceTraveledOnPath() >= distance)
                    .then(command);
        }

        @Override
        public void onInit() {
            buildPaths();
        }

        @Override
        public void onStartButtonPressed() {
            autoRoutine().schedule();
        }

        private Path shoot1, intake1, goBack1, intake2, goBack2, park;

        private final Pose startPose = new Pose(118.000, 130.500, Math.toRadians(216));
        private final Pose scoringPose = new Pose(92.632, 107.117, Math.toRadians(225));

        private final Pose intakePose1 = new Pose(129.150, 83.990, Math.toRadians(0));
        private final Pose intakeShortControlPose1 = new Pose(74.982, 89.224);
        private final Pose intakeShortControlPose2 = new Pose(69.992, 83.990);

        private final Pose intakePose2 = new Pose(128.298, 59.523, Math.toRadians(0));
        private final Pose intakeLongControlPose1 = new Pose(73.765, 89.102);
        private final Pose intakeLongControlPose2 = new Pose(81.555, 58.306);
        private final Pose intakeLongControlPose3 = new Pose(89.467, 59.645);

        private final Pose goBackLongControlPose1 = new Pose(80.825, 70.965);

        private final Pose endPose = new Pose(125.620, 70.600, Math.toRadians(180));


        public void buildPaths() {
            shoot1 = new Path(new BezierLine(startPose, scoringPose));
            shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());

            intake1 = new Path(new BezierCurve(scoringPose, intakeShortControlPose1,intakeShortControlPose2, intakePose1));
            intake1.setTangentHeadingInterpolation();

            goBack1 = new Path(new BezierLine(intakePose1, scoringPose));
            goBack1.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

            intake2 = new Path(new BezierCurve(scoringPose, intakeLongControlPose1, intakeLongControlPose2, intakeLongControlPose3, intakePose2));
            intake2.setTangentHeadingInterpolation();

            goBack2 = new Path(new BezierCurve(intakePose2, goBackLongControlPose1, scoringPose));
            goBack2.setLinearHeadingInterpolation(intakePose2.getHeading(), scoringPose.getHeading());

            park = new Path(new BezierLine(scoringPose, endPose));
            park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
        }

}
