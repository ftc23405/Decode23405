package org.firstinspires.ftc.teamcode.opmodes.autonomous;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "9 Ball Red Side Classifier Auto")
public class RedClassifierAuto extends NextFTCOpMode{

        public RedClassifierAuto() {
            addComponents(
                    new SubsystemComponent(Intake.INSTANCE),
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

        @Override
        public void onInit() {
            buildPaths();
        }

        @Override
        public void onStartButtonPressed() {
            autoRoutine().schedule();
        }

        private Path shoot1, intake1, goBack1, intake2, goBack2, park;

        private final Pose startPose = new Pose(26.000, 129.000, Math.toRadians(-44)).mirror();
        private final Pose scoringPose = new Pose(59.036, 108.578, Math.toRadians(-35)).mirror();

        private final Pose intakePose1 = new Pose(19.354, 83.868, Math.toRadians(180)).mirror();
        private final Pose intakeShortControlPose1 = new Pose(73.278, 98.597).mirror();
        private final Pose intakeShortControlPose2 = new Pose(76.078, 87.398).mirror();
        private final Pose intakeShortControlPose3 = new Pose(63.418, 85.085).mirror();

        private final Pose goBackShortControlPose1 = new Pose(75.347, 82.529).mirror();

        private final Pose intakePose2 = new Pose(18.259, 59.888, Math.toRadians(180)).mirror();
        private final Pose intakeLongControlPose1 = new Pose(94.336, 84.964).mirror();
        private final Pose intakeLongControlPose2 = new Pose(51.855, 62.688).mirror();
        private final Pose intakeLongControlPose3 = new Pose(97.258, 59.645).mirror();

        private final Pose endPose = new Pose(20.693, 69.992, Math.toRadians(0)).mirror();


        public void buildPaths() {
            shoot1 = new Path(new BezierLine(startPose, scoringPose));
            shoot1.setLinearHeadingInterpolation(startPose.getHeading(), scoringPose.getHeading());

            intake1 = new Path(new BezierCurve(scoringPose, intakeShortControlPose1,intakeShortControlPose2, intakeShortControlPose3, intakePose1));
            intake1.setTangentHeadingInterpolation();

            goBack1 = new Path(new BezierCurve(intakePose1, goBackShortControlPose1, scoringPose));
            goBack1.setLinearHeadingInterpolation(intakePose1.getHeading(), scoringPose.getHeading());

            intake2 = new Path(new BezierCurve(scoringPose, intakeLongControlPose1, intakeLongControlPose2, intakeLongControlPose3, intakePose2));
            intake2.setTangentHeadingInterpolation();

            goBack2 = new Path(new BezierLine(intakePose2, scoringPose));
            goBack2.setLinearHeadingInterpolation(intakePose2.getHeading(), scoringPose.getHeading());

            park = new Path(new BezierLine(scoringPose, endPose));
            park.setLinearHeadingInterpolation(scoringPose.getHeading(), endPose.getHeading());
        }

}
