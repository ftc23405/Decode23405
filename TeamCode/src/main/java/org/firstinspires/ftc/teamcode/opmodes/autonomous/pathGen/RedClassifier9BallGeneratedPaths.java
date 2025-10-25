package org.firstinspires.ftc.teamcode.opmodes.autonomous.pathGen;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class RedClassifier9BallGeneratedPaths {

    public static PathBuilder builder = new PathBuilder(null);

    public static PathChain paths = builder
            .addPath(
                    // Path 1
                    new BezierLine(new Pose(118.000, 130.500), new Pose(92.632, 107.117))
            )
            .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(225))
            .addPath(
                    // Path 2
                    new BezierCurve(
                            new Pose(92.632, 107.117),
                            new Pose(74.982, 89.224),
                            new Pose(69.992, 83.990),
                            new Pose(129.150, 83.990)
                    )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                    // Path 3
                    new BezierLine(new Pose(129.150, 83.990), new Pose(92.632, 107.117))
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
            .addPath(
                    // Path 4
                    new BezierCurve(
                            new Pose(92.632, 107.117),
                            new Pose(73.765, 89.102),
                            new Pose(81.555, 58.306),
                            new Pose(89.467, 59.645),
                            new Pose(128.298, 59.523)
                    )
            )
            .setTangentHeadingInterpolation()
            .addPath(
                    // Path 5
                    new BezierCurve(
                            new Pose(128.298, 59.523),
                            new Pose(80.825, 70.965),
                            new Pose(92.876, 107.117)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
            .addPath(
                    // Path 6
                    new BezierLine(new Pose(92.876, 107.117), new Pose(125.620, 70.600))
            )
            .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
            .build();
}
