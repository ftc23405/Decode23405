package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(4.4)
            .forwardZeroPowerAcceleration(-49.94230238)
            .lateralZeroPowerAcceleration(-60.19435624)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.005)

            .translationalPIDFCoefficients(
                    new PIDFCoefficients(0.15, 0.1, 0.1, 0)
            )
            .headingPIDFCoefficients(
                    new PIDFCoefficients(0.7, 0.02, 0.01, 0.1)
            )
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0)
            )
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.2, 0, 0.01, 0)
            )
            .secondaryHeadingPIDFCoefficients(
                    new PIDFCoefficients(0.25, 0.02, 0.01, 0)
            )
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeftMotor")
            .leftRearMotorName("backLeftMotor")
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.117)
            .yVelocity(63.097958)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6)
            .strafePodX(-10)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.0)
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            2.5,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}

