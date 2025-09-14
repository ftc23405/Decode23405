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
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import  org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import  org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//
///**
// * This is an example auto that showcases movement and control of two servos autonomously.
// * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
// * There are examples of different ways to build paths.
// * A path progression method has been created and can advance based on time, position, or other factors.
// *
// * @author Baron Henderson - 20077 The Indubitables
// * @version 2.0, 11/28/2024
// */
//
//@Autonomous
//public class PedroPathingExampleAuto extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//
//    /** This is the variable where we store the state of our auto.
//     * It is used by the pathUpdate method. */
//    private int pathState;
//
//    private Telemetry telemetryA;
//
//    /* Create and Define Poses + Paths
//     * Poses are built with three constructors: x, y, and heading (in Radians).
//     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
//     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
//     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
//     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
//     * Lets assume our robot is 18 by 18 inches
//     * Lets assume the Robot is facing the human player and we want to score in the bucket */
//
//    /** Start Pose of our robot */
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
//    /* These are our Paths and PathChains that we will define in buildPaths() */
//    private Path scoreSpec1, park;
//    private PathChain goToRight, pushSample1, pushSample2, pushSample3, pickUpSpecAfterPushingSamples, scoreSpec2, goToZoneGetSpec3, scoreSpec3, goToZoneGetSpec4, scoreSpec4, goToZoneGetSpec5, scoreSpec5;
//
//    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
//     * It is necessary to do this so that all the paths are built before the auto starts. **/
//    public void buildPaths() {
//
//
//        scoreSpec1 = new Path(new BezierLine(new Point(startPose), new Point(scoreSpec1Pose)));
//        scoreSpec1.setConstantHeadingInterpolation(startPose.getHeading());
//
//        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        goToRight = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(scoreSpec1Pose), new Point(getGoToRightControlPose1), new Point(goToRightPose)))
//                .setLinearHeadingInterpolation(scoreSpec1Pose.getHeading(), goToRightPose.getHeading())
//                .build();
//
//        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        pushSample1 = follower.pathBuilder()
//                .addPath(new BezierCurve(new Point(goToRightPose), new Point(pushSample1ControlPose1), new Point(pushSample1Pose)))
//                .setConstantHeadingInterpolation(pushSample1Pose.getHeading())
//                .build();
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
//    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
//     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
//     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                follower.followPath(scoreSpec1);
//                setPathState(1);
//                break;
//            case 1:
//
//                /* You could check for
//                - Follower State: "if(!follower.isBusy() {}"
//                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
//                - Robot Position: "if(follower.getPose().getX() > 36) {}"
//                */
//
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Preload */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(goToRight,true);
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(pushSample1,true);
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(pushSample2,true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(pushSample3,true);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(pickUpSpecAfterPushingSamples,true);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scoreSpec2, true);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(goToZoneGetSpec3,true);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(scoreSpec3,true);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(goToZoneGetSpec4,true);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(scoreSpec4,true);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(goToZoneGetSpec5,true);
//                    setPathState(12);
//                }
//                break;
//            case 12:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(scoreSpec5,true);
//                    setPathState(13);
//                }
//                break;
//
//            case 13:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
//                    follower.followPath(park,true);
//                    setPathState(14);
//                }
//                break;
//            case 14:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Level 1 Ascent */
//
//                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
//                    setPathState(-1);
//                }
//                break;
//        }
//    }
//
//    /** These change the states of the paths and actions
//     * It will also reset the timers of the individual switches **/
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
//    @Override
//    public void loop() {
//
//        // These loop the movements of the robot
//        follower.update();
//        autonomousPathUpdate();
//
//        // Feedback to Driver Hub
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetryA.update();
//        telemetry.update();
//    }
//
//    /** This method is called once at the init of the OpMode. **/
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        opmodeTimer = new Timer();
//        opmodeTimer.resetTimer();
//
//        follower.telemetryDebug(telemetryA);
//
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    /** This method is called continuously after Init while waiting for "play". **/
//    @Override
//    public void init_loop() {}
//
//    /** This method is called once at the start of the OpMode.
//     * It runs all the setup actions, including building paths and starting the path system **/
//    @Override
//    public void start() {
//        opmodeTimer.resetTimer();
//        setPathState(0);
//    }
//
//    /** We do not use this because everything should automatically disable **/
//    @Override
//    public void stop() {
//    }
//}