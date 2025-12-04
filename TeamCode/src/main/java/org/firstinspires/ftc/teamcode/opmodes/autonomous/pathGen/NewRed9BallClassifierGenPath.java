package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathGen;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class NewRed9BallClassifierGenPath {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;

    public NewRed9BallClassifierGenPath(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111.000, 135.000), new Pose(92.000, 94.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 94.000), new Pose(92.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 84.000), new Pose(129.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(129.000, 84.000), new Pose(92.000, 94.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 94.000), new Pose(92.000, 59.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 59.000), new Pose(135.000, 59.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(135.000, 59.000),
                                new Pose(88.000, 67.000),
                                new Pose(92.000, 94.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(230))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(92.000, 94.000), new Pose(129.000, 94.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(0))
                .build();
    }
}