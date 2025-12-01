package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathGen;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class NewRed9BallFarGenPath {

    public static PathBuilder builder = new PathBuilder(null);

    public static PathChain Path1 = builder
            .addPath(new BezierLine(new Pose(82.000, 9.000), new Pose(85.000, 22.000)))
            .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(250))
            .build();

    public static PathChain Path2 = builder
            .addPath(
                    new BezierLine(new Pose(85.000, 22.000), new Pose(117.000, 18.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
            .build();

    public static PathChain Path3 = builder
            .addPath(
                    new BezierLine(new Pose(117.000, 18.000), new Pose(132.000, 18.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-20))
            .build();

    public static PathChain Path4 = builder
            .addPath(
                    new BezierLine(new Pose(132.000, 18.000), new Pose(134.000, 8.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(-40))
            .build();

    public static PathChain Path5 = builder
            .addPath(new BezierLine(new Pose(134.000, 8.000), new Pose(85.000, 22.000)))
            .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(250))
            .build();

    public static PathChain Path6 = builder
            .addPath(new BezierLine(new Pose(85.000, 22.000), new Pose(85.000, 36.000)))
            .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
            .build();

    public static PathChain Path7 = builder
            .addPath(
                    new BezierLine(new Pose(85.000, 36.000), new Pose(135.000, 36.000))
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();

    public static PathChain Path8 = builder
            .addPath(
                    new BezierLine(new Pose(135.000, 36.000), new Pose(85.000, 22.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(250))
            .build();

    public static PathChain Path9 = builder
            .addPath(
                    new BezierLine(new Pose(85.000, 22.000), new Pose(135.000, 36.000))
            )
            .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(0))
            .build();
}
