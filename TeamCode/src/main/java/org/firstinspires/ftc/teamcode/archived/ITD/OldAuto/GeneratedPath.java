//package org.firstinspires.ftc.teamcode.opmodes.autonomous;
//
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathBuilder;
//import com.pedropathing.pathgen.Point;
//
//public class GeneratedPath {
//
//    public GeneratedPath() {
//        PathBuilder builder = new PathBuilder();
//
//        builder
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(9.000, 56.100, Point.CARTESIAN),
//                                new Point(40.000, 72.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(-180))
//                .addPath(
//                        // Line 2
//                        new BezierCurve(
//                                new Point(40.000, 72.000, Point.CARTESIAN),
//                                new Point(21.000, 75.000, Point.CARTESIAN),
//                                new Point(36.000, 37.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(0))
//                .addPath(
//                        // Line 3
//                        new BezierCurve(
//                                new Point(36.000, 37.000, Point.CARTESIAN),
//                                new Point(100.100, 29.400, Point.CARTESIAN),
//                                new Point(14.000, 22.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 4
//                        new BezierCurve(
//                                new Point(14.000, 22.000, Point.CARTESIAN),
//                                new Point(107.300, 30.000, Point.CARTESIAN),
//                                new Point(39.000, 4.600, Point.CARTESIAN),
//                                new Point(72.300, 16.400, Point.CARTESIAN),
//                                new Point(14.800, 10.400, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 5
//                        new BezierCurve(
//                                new Point(14.800, 10.400, Point.CARTESIAN),
//                                new Point(99.200, 18.200, Point.CARTESIAN),
//                                new Point(56.000, 5.100, Point.CARTESIAN),
//                                new Point(16.300, 7.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 6
//                        new BezierCurve(
//                                new Point(16.300, 7.000, Point.CARTESIAN),
//                                new Point(48.600, 17.600, Point.CARTESIAN),
//                                new Point(8.000, 24.000, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(
//                        // Line 7
//                        new BezierCurve(
//                                new Point(8.000, 24.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(40.000, 72.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
//                .addPath(
//                        // Line 8
//                        new BezierCurve(
//                                new Point(40.000, 72.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(8.000, 24.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .addPath(
//                        // Line 9
//                        new BezierCurve(
//                                new Point(8.000, 24.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(40.000, 72.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
//                .addPath(
//                        // Line 10
//                        new BezierCurve(
//                                new Point(40.000, 72.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(8.000, 24.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .addPath(
//                        // Line 11
//                        new BezierCurve(
//                                new Point(8.000, 24.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(40.000, 72.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
//                .addPath(
//                        // Line 12
//                        new BezierCurve(
//                                new Point(40.000, 72.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(8.000, 24.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .addPath(
//                        // Line 13
//                        new BezierCurve(
//                                new Point(8.000, 24.000, Point.CARTESIAN),
//                                new Point(48.500, 40.000, Point.CARTESIAN),
//                                new Point(14.200, 77.000, Point.CARTESIAN),
//                                new Point(40.000, 72.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
//                .addPath(
//                        // Line 14
//                        new BezierCurve(
//                                new Point(40.000, 72.000, Point.CARTESIAN),
//                                new Point(19.719, 71.452, Point.CARTESIAN),
//                                new Point(13.000, 18.000, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));
//    }
//}