package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.paths.HeadingInterpolator.linearFromPoint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.control.KalmanFilterParameters;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorLeft;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorRight;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.TransferPusher;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.tuning.KalmanFilterPlus;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static org.firstinspires.ftc.teamcode.pedroPathing.Drawing.drawRobot;
import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

@TeleOp
@Configurable //panels
public class V5_Teleop extends NextFTCOpMode {

    public V5_Teleop() {
        addComponents( //add needed components
                new SubsystemComponent(Intake.INSTANCE, ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                new SubsystemComponent(TransferPusher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                CommandManager.INSTANCE,
                new PedroComponent(Constants::createFollower) //follower for Pedro teleop drive
        );
    }

    private final KalmanFilterPlus kfX = new KalmanFilterPlus(new KalmanFilterParameters(0.5, 0.5));
    private final KalmanFilterPlus kfY = new KalmanFilterPlus(new KalmanFilterParameters(0.5, 0.5));
    private final KalmanFilterPlus kfTh = new KalmanFilterPlus(new KalmanFilterParameters(0.5, 0.5));

    private double ppBotX;
    private double ppBotY;
    private double ppBotHeading;

    private PathChain parkPath;

    PIDFController headingController = new PIDFController(null);
    Button parkButton = (Gamepads.gamepad1().dpadUp()).and(Gamepads.gamepad2().dpadUp());

    private Pose redParkPose = new Pose(39, 33, Math.toRadians(180));

    private double llBotX;
    private double llBotY;
    private double llBotTh;

    private InterpLUT interpLUT = new InterpLUT();
    private double distance;

    private Limelight3A limelight;


    boolean headingLock = false;


    @Override
    public void onInit() {
        interpLUT.add(0,0);
        interpLUT.add(1,2);
        interpLUT.add(3,5);
        interpLUT.createLUT();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        PedroComponent.follower().setStartingPose(new Pose(0,0, Math.toRadians(0))); //set starting pose for pinpoint IMU

        parkPath = PedroComponent.follower().pathBuilder()
                .addPath(new Path(new BezierLine(PedroComponent.follower()::getPose, redParkPose)))
                .setHeadingInterpolation(linearFromPoint(PedroComponent.follower()::getHeading, redParkPose.getHeading(), 0.8))
                .build();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());

    }


    @Override
    public void onStartButtonPressed() {
        PedroComponent.follower().startTeleopDrive();
        limelight.start();
        Gamepads.gamepad1().start()
                .whenBecomesTrue(new SequentialGroup(
                        new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(180)))),
                        new InstantCommand(() -> gamepad1.rumble(500))
                )); //reset pinpoint IMU

        Gamepads.gamepad1().y()
                .whenTrue(() -> headingLock = true)
                .whenFalse(() -> headingLock = false);

        Gamepads.gamepad1().a()
                .whenBecomesTrue(new ParallelGroup(
                        ShooterMotorLeft.INSTANCE.autoRPM(interpLUT.get(distance)),
                        ShooterMotorRight.INSTANCE.autoRPM(interpLUT.get(distance))
                ));

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new ParallelGroup(
                        ShooterMotorLeft.INSTANCE.shooterMotorLeftClassifier(),
                        ShooterMotorRight.INSTANCE.shooterMotorRightClassifier()
                ));
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(new ParallelGroup(
                        ShooterMotorLeft.INSTANCE.shooterMotorLeftFar(),
                        ShooterMotorRight.INSTANCE.shooterMotorRightFar()
                ));
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(new ParallelGroup(
                        ShooterMotorLeft.INSTANCE.shooterMotorLeftOff(),
                        ShooterMotorRight.INSTANCE.shooterMotorRightOff()
                ));

        Gamepads.gamepad2().rightBumper().whenBecomesTrue(() -> {
                    if (ShooterMotorRight.INSTANCE.isAtTarget(50)) {
                        new SequentialGroup(
                                new ParallelGroup(Intake.INSTANCE.intakeAutoSpeed, TransferPusher.INSTANCE.transferPush)
                        ).schedule();
                    }
                })
                .whenBecomesFalse(TransferPusher.INSTANCE.transferHold); //when button held transfer runs, when let go go back to hold position


        Gamepads.gamepad2().y()
                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad2().x()
                .whenBecomesTrue(Intake.INSTANCE.intakeHalfSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad2().a()
                .whenBecomesTrue(Intake.INSTANCE.intakeOneThirdSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);
        Gamepads.gamepad2().b()
                .whenBecomesTrue(Intake.INSTANCE.intakeReverseFullSpeed)
                .whenBecomesFalse(Intake.INSTANCE.intakeOff);


        parkButton
                .whenBecomesTrue(new FollowPath(parkPath));

    }


    @Override
    public void onUpdate() { //runs every loop

//        if ((Math.abs(Math.toDegrees(PedroComponent.follower().getPose().getHeading())) <= 2) ){ //if follower has heading of 180 degrees (with 2 degrees of tolerance), reset the IMU
//            new InstantCommand(() -> PedroComponent.follower().setPose(PedroComponent.follower().getPose().withHeading(Math.toRadians(0)))); //reset pinpoint IMU);
//        }

        BindingManager.update();
        LLResult llResult = limelight.getLatestResult();
        double targetHeading = Math.toRadians(-llResult.getTx()); // Radians

        double error = targetHeading;
        headingController.setCoefficients(new PIDFCoefficients(headingP, headingI, headingD, headingFF));
        headingController.updateError(error);

        if (headingLock && llResult.isValid())
            PedroComponent.follower().setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, headingController.run(), false);
        else
            PedroComponent.follower().setTeleOpDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

        distance = getDistanceFromTag(llResult.getTy());

        telemetry.addData("limelight tx", llResult.getTx());
        telemetry.addData("Distance from Tag", distance);

//        PedroComponent.follower().setPose(getRobotPoseFromLL());

        ppBotX = PedroComponent.follower().getPose().getX();
        ppBotY = PedroComponent.follower().getPose().getY();
        ppBotHeading = PedroComponent.follower().getPose().getHeading();
//        double filteredX = getRobotPoseFromLL().getX();
//        double filteredY = getRobotPoseFromLL().getY();
//        double filteredTh = getRobotPoseFromLL().getHeading();


        telemetry.addData("Robot Heading", Math.toDegrees(ppBotHeading));
        telemetry.addData("Robot x", ppBotX);
        telemetry.addData("Robot y", ppBotY);

//        telemetry.addData("Filtered Heading", Math.toDegrees(filteredTh));
//        telemetry.addData("Filtered x", filteredX);
//        telemetry.addData("Filtered y", filteredY);


        telemetry.update();
//        drawRobot(PedroComponent.follower().getPose(), kfX, kfY, kfTh);
    }

    private Pose getRobotPoseFromLL() {
        // Latest Limelight result
        LLResult llResult = limelight.getLatestResult();
        Pose llPosePedro = null;

        // --- LIMELIGHT POSE EXTRACTION ---
        if (llResult != null && llResult.isValid()) {
            Pose3D llBotPose = llResult.getBotpose();

            // Convert FTC -> Pedro coordinate system
            llPosePedro = new Pose(
                    llBotPose.getPosition().x,
                    llBotPose.getPosition().y,
                    llBotPose.getOrientation().getYaw(AngleUnit.RADIANS),
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            // Debug telemetry
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", llBotPose);
            telemetry.addData("Yaw", llBotPose.getOrientation().getYaw());
        }

        // --- ODOMETRY INPUTS ---
        Pose odomDelta = PedroComponent.follower().poseTracker.getDeltaPose();
        Pose prevPose  = PedroComponent.follower().poseTracker.getPreviousPose();

        // --- SENSOR FUSION ---
        if (llPosePedro != null) {
            // Limelight delta relative to previous pose
            Pose llDelta = llPosePedro.minus(prevPose);

            kfX.update(odomDelta.getX(), llDelta.getX());
            kfY.update(odomDelta.getY(), llDelta.getY());
            kfTh.update(odomDelta.getHeading(), llDelta.getHeading());
        } else {
            // No LL update → use odometry only
            kfX.update(odomDelta.getX());
            kfY.update(odomDelta.getY());
            kfTh.update(odomDelta.getHeading());
        }

        // --- RETURN FILTERED POSE (Pedro coordinates) ---
        return new Pose(
                kfX.getState(),
                kfY.getState(),
                kfTh.getState()
        );
    }

    public double getDistanceFromTag(double ty) {
        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        limelight.stop();
    }
}
