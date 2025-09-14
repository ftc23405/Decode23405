//package org.firstinspires.ftc.teamcode.opmodes.autonomous;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathBuilder;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//
//import java.util.Timer;
//
//public class GeneratedSamplePath extends OpMode {
//    private Timer pathTimer;
//    private Follower follower;
//    private Pose startPose = new Pose(8, 80, 0);
//
//    public static PathBuilder builder = new PathBuilder();
//
//    public static PathChain line1 = builder
//            .addPath(new BezierLine(new Point(8.000, 80.000, Point.CARTESIAN), new Point(11.664, 115.065, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//            .build();
//
//    public static PathChain line2 = builder
//            .addPath(new BezierLine(new Point(11.664, 115.065, Point.CARTESIAN), new Point(44.187, 121.570, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
//            .build();
//
//    public static PathChain line3 = builder
//            .addPath(new BezierLine(new Point(44.187, 121.570, Point.CARTESIAN), new Point(9.645, 136.374, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(100))
//            .build();
//
//    public static PathChain line4 = builder
//            .addPath(new BezierLine(new Point(9.645, 136.374, Point.CARTESIAN), new Point(41.271, 131.888, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(30))
//            .build();
//
//    public static PathChain line5 = builder
//            .addPath(new BezierLine(new Point(41.271, 131.888, Point.CARTESIAN), new Point(9.421, 135.701, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(110))
//            .build();
//
//    public static PathChain line7 = builder
//            .addPath(new BezierLine(new Point(44.860, 141.084, Point.CARTESIAN), new Point(9.421, 135.701, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(100))
//            .build();
//
//    public static PathChain line8 = builder
//            .addPath(new BezierLine(new Point(9.421, 135.701, Point.CARTESIAN), new Point(70.879, 96.224, Point.CARTESIAN)))
//            .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180))
//            .build();
//
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//        telemetry.addData("Position", follower.getPose().toString());
//        telemetry.update();
//    }
//
//    private void buildPaths() {
//        // Implementation for buildPaths
//    }
//
//    private void autonomousPathUpdate() {
//        // Implementation for autonomousPathUpdate
//    }
//}
