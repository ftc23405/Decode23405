package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathGen;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class RedFar6BallGeneratedPaths {

    public static PathBuilder builder = new PathBuilder(null);

    public static PathChain paths = builder
            .addPath(
                    // Path 1
                    new BezierLine(new Pose(82.017, 7.096), new Pose(84.233, 18.989))
            )
            .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-116))
            .addPath(
                    // Path 2
                    new BezierLine(new Pose(84.233, 18.989), new Pose(97.461, 34.435))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-116), Math.toRadians(0))
            .addPath(
                    // Path 3
                    new BezierLine(new Pose(97.461, 34.435), new Pose(131.687, 34.852))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(
                    // Path 4
                    new BezierLine(new Pose(131.687, 34.852), new Pose(84.355, 18.867))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-116))
            .addPath(
                    // Path 5
                    new BezierLine(new Pose(84.355, 18.867), new Pose(105.413, 32.987))
            )
            .setLinearHeadingInterpolation(Math.toRadians(-116), Math.toRadians(180))
            .build();
}