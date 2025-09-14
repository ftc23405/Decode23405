//package org.firstinspires.ftc.teamcode.opmodes.teleop;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.rowanmcalpin.nextftc.core.command.Command;
//import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
//import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
//import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
//import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
//import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.DepositBucket;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeArm;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeArmWrist;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeClaw;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.Slides;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.SpecClaw;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.*;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//@TeleOp
//public class NextFtcTeleop extends NextFTCOpMode {
//
//    public NextFtcTeleop() {
//        super(DepositBucket.INSTANCE, IntakeClaw.INSTANCE, Slides.INSTANCE, SpecClaw.INSTANCE, IntakeArm.INSTANCE, IntakeArmWrist.INSTANCE);
//    }
//
//    public String backLeftName = "backLeftMotor";
//    public String frontRightName = "frontRightMotor";
//    public String backRightName = "backRightMotor";
//    public String frontLeftName = "frontLeftMotor";
//
//    public MotorEx frontLeftMotor;
//    public MotorEx frontRightMotor;
//    public MotorEx backLeftMotor;
//    public MotorEx backRightMotor;
//
//    public MotorEx[] motors;
//
//    public IMU imu;
//
//    public Command driverControlled;
//
//    private OpenCvCamera controlHubCam;
//    private static final int CAMERA_WIDTH = 640;
//    private static final int CAMERA_HEIGHT = 360;
//
//    public static final double objectWidthInRealWorldUnits = 3.75;
//    public static final double focalLength = 728;
//
//    // OpenCV related variables (these are now public)
//    public String detectedColor = "";  // To store detected color (Red, Yellow, Blue)
//    public double detectedCoordX = 0.0;  // X coordinate of detected object
//    public double detectedCoordY = 0.0;  // Y coordinate of detected object
//    public double detectedWidth = 0.0;  // Width of detected object
//    public double detectedDistance = 0.0;  // Calculated distance of detected object
//
//    @Override
//    public void onInit() {
//        // Initialize motors
//        frontLeftMotor = new MotorEx(frontLeftName);
//        backLeftMotor = new MotorEx(backLeftName);
//        backRightMotor = new MotorEx(backRightName);
//        frontRightMotor = new MotorEx(frontRightName);
//
//        // Set motor directions
//        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Motor behaviors
//        backLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeftMotor.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        motors = new MotorEx[] { frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor };
//
//        // Initialize IMU for field centric control
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
//        imu.resetYaw();
//
//        initOpenCV(); // Initialize OpenCV for color detection
//    }
//
//    private void initOpenCV() {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
//                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        controlHubCam.setPipeline(new ColorDetectionPipeline());  // Corrected to set pipeline to an instance of ColorDetectionPipeline
//        controlHubCam.openCameraDevice();
//        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1(), false, imu);
//        driverControlled.invoke();
//
//        // Add your commands for control buttons
//        gamepadManager.getGamepad1().getRightBumper().setPressedCommand(IntakeClaw.INSTANCE::full_close);
//        gamepadManager.getGamepad1().getLeftBumper().setPressedCommand(IntakeClaw.INSTANCE::transfer_intake_open);
//
//        gamepadManager.getGamepad2().getX().setPressedCommand(SpecClaw.INSTANCE::open);
//        gamepadManager.getGamepad2().getB().setPressedCommand(SpecClaw.INSTANCE::close);
//        gamepadManager.getGamepad2().getY().setPressedCommand(DepositBucket.INSTANCE::ToDeposit);
//        gamepadManager.getGamepad2().getA().setPressedCommand(DepositBucket.INSTANCE::ToTransfer);
//
//        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(Slides.INSTANCE::toDeposit);
//        gamepadManager.getGamepad2().getDpadDown().setPressedCommand(Slides.INSTANCE::toLowest);
//        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(Slides.INSTANCE::toSpecBar);
//        gamepadManager.getGamepad2().getDpadRight().setPressedCommand(Slides.INSTANCE::toSpecClip);
//        gamepadManager.getGamepad2().getBack().setPressedCommand(Slides.INSTANCE::resetSlidesEncoder);
//
//        // Control for intake arm and wrist (example)
//        gamepadManager.getGamepad1().getA().setPressedCommand(
//                () -> new SequentialGroup(
//                        IntakeArm.INSTANCE.toGrab(),
//                        IntakeArmWrist.INSTANCE.grab(),
//                        IntakeClaw.INSTANCE.full_open()
//                )
//        );
//    }
//
//    @Override
//    public void onUpdate() {
//        // Get the detected color and coordinates from OpenCV pipeline
//        detectedColor = ColorDetectionPipeline.getDetectedColor();  // This gets the color detected (Red, Yellow, Blue)
//        detectedCoordX = ColorDetectionPipeline.cX;  // X coordinate of the detected object
//        detectedCoordY = ColorDetectionPipeline.cY;  // Y coordinate of the detected object
//        detectedWidth = ColorDetectionPipeline.width; // Width of the detected object for distance calculation
//        detectedDistance = ColorDetectionPipeline.getDistance(detectedWidth); // Distance of the detected object in inches
//
//        // Update telemetry with the detected color, coordinates, and distance
//        telemetry.addData("Detected Color", detectedColor);
//        telemetry.addData("Coordinate", "(" + (int) detectedCoordX + ", " + (int) detectedCoordY + ")");
//        telemetry.addData("Distance in Inches", detectedDistance);
//        telemetry.update();
//    }
//
//    // Color detection pipeline class embedded within NextFtcTeleop
//    public static class ColorDetectionPipeline extends OpenCvPipeline {
//
//        public static double cX;
//        public static double cY;
//        public static double width;
//        public static String detectedColor;
//
//        public static double getDistance(double width) {
//            return 100.0 / width;  // Example formula for distance calculation
//        }
//
//        public static String getDetectedColor() {
//            return detectedColor;
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            Scalar lowerBound = new Scalar(0, 100, 100);  // Example for Red color
//            Scalar upperBound = new Scalar(10, 255, 255);
//
//            // Convert to HSV
//            Mat hsvMat = new Mat();
//            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_BGR2HSV);
//
//            // Create a mask for the specified color
//            Mat mask = new Mat();
//            Core.inRange(hsvMat, lowerBound, upperBound, mask);
//
//            // Find contours of the detected object
//            java.util.ArrayList<MatOfPoint> contours = new java.util.ArrayList<>();
//            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            // Assuming we found a contour
//            if (contours.size() > 0) {
//                // Get the bounding rectangle of the largest contour
//                MatOfPoint largestContour = contours.get(0);
//                Rect boundingRect = Imgproc.boundingRect(largestContour);
//
//                cX = boundingRect.x + boundingRect.width / 2; // X coordinate of detected object
//                cY = boundingRect.y + boundingRect.height / 2; // Y coordinate of detected object
//                width = boundingRect.width;  // Width of the detected object
//                detectedColor = "Red";  // Example color (you can add more color detection logic)
//            }
//
//            return input;
//        }
//    }
//}
