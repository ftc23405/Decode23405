//package org.firstinspires.ftc.teamcode.opmodes.autonomous;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
//import com.rowanmcalpin.nextftc.pedro.FollowPath;
//import com.rowanmcalpin.nextftc.pedro.PedroOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositBucket;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeArm;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeArmWrist;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.Slides;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.SpecClaw;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//
//@Autonomous(name = "monkeymonkey")
//public class Pedro5Spec extends PedroOpMode {
//
//    public Pedro5Spec() {
//        super(DepositBucket.INSTANCE, IntakeArm.INSTANCE, IntakeArmWrist.INSTANCE, IntakeClaw.INSTANCE, Slides.INSTANCE, SpecClaw.INSTANCE);
//    }
//
//    public Command firstRoutine() {
//        return new SequentialGroup(
//                new SequentialGroup(
//                        SpecClaw.INSTANCE.close()
//                ),
//                new Delay(0.1),
//                new ParallelGroup(
//                        new FollowPath(scoreSpec1,true),
//                        Slides.INSTANCE.toSpecBar()
//                ),
//                new SequentialGroup(
//                      Slides.INSTANCE.toSpecClip(),
//                        SpecClaw.INSTANCE.open()
//                )
//        );
//    }
//
//    public Command secondRoutine() {
//        return new SequentialGroup(
//                new SequentialGroup(new FollowPath(goToRight)),
//                Slides.INSTANCE.toLowest(),
//                new FollowPath(pushSample1),
//                new FollowPath(pushSample2,true),
//                new FollowPath(pushSample3,true),
//                new FollowPath(pickUpSpecAfterPushingSamples,true)
//        );
//    }
//
//    public Command thirdRoutine() {
//        return new SequentialGroup(
//                SpecClaw.INSTANCE.close(),
//                new Delay(0.1),
//                Slides.INSTANCE.toSpecBar(),
//                new SequentialGroup(new FollowPath(scoreSpec2,true)),
//                new Delay(0.2),
//                Slides.INSTANCE.toSpecClip(),
//                new Delay(0.1),
//                SpecClaw.INSTANCE.open()
//        );
//    }
//
//    public Command fourthRoutine() {
//        return new SequentialGroup(
//            new ParallelGroup(
//                    new FollowPath(goToZoneGetSpec3,true),
//                    Slides.INSTANCE.toLowest()
//            ),
//                new SequentialGroup(
//                        SpecClaw.INSTANCE.close(),
//                        new Delay(0.1),
//                        Slides.INSTANCE.toSpecBar()
//                )
//        );
//    }
//
//    private Telemetry telemetry1;
//
//    private final Pose startPose = new Pose(9, 56.1, Math.toRadians(180));
//
//
//    private final Pose scoreSpec1Pose = new Pose(40, 72, Math.toRadians(180));
//
//
//    private final Pose goToRightPose = new Pose(36, 37, Math.toRadians(0));
//
//    private final Pose getGoToRightControlPose1 = new Pose(21,75, Math.toRadians(0));
//
//
//    private final Pose pushSample1Pose = new Pose(14, 22, Math.toRadians(0));
//
//    private final Pose pushSample1ControlPose1 = new Pose(100.1, 29.4, Math.toRadians(0));
//
//
//    private final Pose pushSample2Pose = new Pose(14.8, 10.4, Math.toRadians(0));
//
//    private final Pose pushSample2ControlPose1 = new Pose(107.3, 30, Math.toRadians(0));
//
//    private final Pose pushSample2ControlPose2 = new Pose(39, 4.6, Math.toRadians(0));
//
//    private final Pose pushSample2ControlPose3 = new Pose(72.3, 16.4, Math.toRadians(0));
//
//
//    private final Pose pushSample3Pose = new Pose(16.3, 7, Math.toRadians(0));
//
//    private final Pose pushSample3ControlPose1 = new Pose(99.2, 18.2, Math.toRadians(0));
//
//    private final Pose pushSample3ControlPose2 = new Pose(56, 5.1, Math.toRadians(0));
//
//
//    private final Pose specPickupPose = new Pose(8,24, Math.toRadians(0));
//
//    private final Pose pickUpAfterPushingControlPose1 = new Pose(48.6,17.6, Math.toRadians(0));
//
//
//    private final Pose scoreOtherSpecsFromZonePose = new Pose(40,72, Math.toRadians(180));
//
//    private final Pose scoreOtherSpecsFromZoneControlPose1 = new Pose(48.5,40, Math.toRadians(180));
//
//    private final Pose scoreOtherSpecsFromZoneControlPose2 = new Pose(14.2,77, Math.toRadians(180));
//
//
//    private final Pose goBackToZonePose = new Pose(8,24, Math.toRadians(0));
//
//    private final Pose goBackToZoneControlPose1 = new Pose(14.2,77,Math.toRadians(0));
//
//    private final Pose goBackToZoneControlPose2 = new Pose(48.5,40,Math.toRadians(0));
//
//
//    private final Pose parkPose = new Pose(13, 18, Math.toRadians(0));
//
//    private final Pose parkControlPose = new Pose(19.719, 71.452, Math.toRadians(0));
//
//    private Path scoreSpec1, park, goToRight, pushSample1;
//    private PathChain pushSample2, pushSample3, pickUpSpecAfterPushingSamples, scoreSpec2, goToZoneGetSpec3, scoreSpec3, goToZoneGetSpec4, scoreSpec4, goToZoneGetSpec5, scoreSpec5;
//
//    public void buildPaths() {
//
//        scoreSpec1 = new Path(new BezierLine(new Point(startPose), new Point(scoreSpec1Pose)));
//        scoreSpec1.setConstantHeadingInterpolation(startPose.getHeading());
//
//        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        goToRight = new Path(new BezierCurve(new Point(scoreSpec1Pose), new Point(getGoToRightControlPose1), new Point(goToRightPose)));
//        goToRight.setLinearHeadingInterpolation(scoreSpec1Pose.getHeading(), goToRightPose.getHeading());
//
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        pushSample1 = new Path(new BezierCurve(new Point(goToRightPose), new Point(pushSample1ControlPose1), new Point(pushSample1Pose)));
//        pushSample1.setConstantHeadingInterpolation(pushSample1Pose.getHeading());
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        pushSample2 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(pushSample1Pose), new Point(pushSample2ControlPose1), new Point(pushSample2ControlPose2), new Point(pushSample2ControlPose3), new Point(pushSample2Pose)))
//                .setConstantHeadingInterpolation(pushSample2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        pushSample3 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(pushSample2Pose), new Point(pushSample3ControlPose1), new Point(pushSample3ControlPose2), new Point(pushSample3Pose)))
//                .setConstantHeadingInterpolation(pushSample3Pose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        pickUpSpecAfterPushingSamples = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(pushSample3Pose), new Point(pickUpAfterPushingControlPose1), new Point(specPickupPose)))
//                .setConstantHeadingInterpolation(specPickupPose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scoreSpec2 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(specPickupPose), new Point(scoreOtherSpecsFromZoneControlPose1), new Point(scoreOtherSpecsFromZoneControlPose2), new Point(scoreOtherSpecsFromZonePose)))
//                .setLinearHeadingInterpolation(specPickupPose.getHeading(), scoreOtherSpecsFromZonePose.getHeading())
//                .build();
//
//        goToZoneGetSpec3 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scoreOtherSpecsFromZonePose), new Point(goBackToZoneControlPose1), new Point(goBackToZoneControlPose2), new Point(goBackToZonePose)))
//                .setLinearHeadingInterpolation(scoreOtherSpecsFromZonePose.getHeading(), goBackToZonePose.getHeading())
//                .build();
//
//        scoreSpec3 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(goBackToZonePose), new Point(scoreOtherSpecsFromZoneControlPose1), new Point(scoreOtherSpecsFromZoneControlPose2), new Point(scoreOtherSpecsFromZonePose)))
//                .setLinearHeadingInterpolation(goBackToZonePose.getHeading(), scoreOtherSpecsFromZonePose.getHeading())
//                .build();
//
//        goToZoneGetSpec4 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scoreOtherSpecsFromZonePose), new Point(goBackToZoneControlPose1), new Point(goBackToZoneControlPose2), new Point(goBackToZonePose)))
//                .setLinearHeadingInterpolation(scoreOtherSpecsFromZonePose.getHeading(), goBackToZonePose.getHeading())
//                .build();
//
//        scoreSpec4 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(goBackToZonePose), new Point(scoreOtherSpecsFromZoneControlPose1), new Point(scoreOtherSpecsFromZoneControlPose2), new Point(scoreOtherSpecsFromZonePose)))
//                .setLinearHeadingInterpolation(goBackToZonePose.getHeading(), scoreOtherSpecsFromZonePose.getHeading())
//                .build();
//
//        goToZoneGetSpec5 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scoreOtherSpecsFromZonePose), new Point(goBackToZoneControlPose1), new Point(goBackToZoneControlPose2), new Point(goBackToZonePose)))
//                .setLinearHeadingInterpolation(scoreOtherSpecsFromZonePose.getHeading(), goBackToZonePose.getHeading())
//                .build();
//
//        scoreSpec5 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(goBackToZonePose), new Point(scoreOtherSpecsFromZoneControlPose1), new Point(scoreOtherSpecsFromZoneControlPose2), new Point(scoreOtherSpecsFromZonePose)))
//                .setLinearHeadingInterpolation(goBackToZonePose.getHeading(), scoreOtherSpecsFromZonePose.getHeading())
//                .build();
//
//        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(scoreOtherSpecsFromZonePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scoreOtherSpecsFromZonePose.getHeading(), parkPose.getHeading());
//    }
//
//    @Override
//    public void onInit() {
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        buildPaths();
//
//    }
//
//    @Override
//    public void onUpdate() {
//
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.update();
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        new SequentialGroup(
//                firstRoutine(),
//                secondRoutine(),
//                thirdRoutine()
//        ).invoke();
//
//    }
//}
