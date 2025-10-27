package org.firstinspires.ftc.teamcode.commandbase.vision;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous
public class AprilTagWebcamExample extends NextFTCOpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private TelemetryManager telemetryM;

    @Override
    public void onInit() {
        aprilTagWebcam.init(hardwareMap, telemetryM);
    }

    @Override
    public void onUpdate() {
        //update vision portal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
    }
}
