package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.tuning.PIDFMotorController;

public class India_Editable_Auto extends LinearOpMode {

    public static double MAX_ARM_POWER = 0.7;
    public static int ARM_INITIAL_ANGLE = 90; //deg
    public static int ARM_INTAKE_POSITION = 2250;
    public static int ARM_UP_POSITION = 0;
    public static double INTAKE_POWER = 0.75;
    public static double OUTTAKE_POWER = 1;
    public static int OUTTAKE_POS = 500;
    public static double DRIVETRAIN_POWER = 0.75;
    private PIDFMotorController armController;
    private DcMotor rightMotor, leftMotor;
    private com.qualcomm.robotcore.hardware.CRServo intakeServo;

    @Override
    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        DcMotorEx intakeArmMotor = hardwareMap.get(DcMotorEx.class, "intakeArmMotor");

        intakeServo = hardwareMap.get(CRServo .class, "intakeServo");

        final double armTicksInDegrees = 537.7 / 360.0;

        // Initialize PIDF controllers for the arm
        armController = new PIDFMotorController(intakeArmMotor, 0.01, 0.25, 0.001, 0.4, armTicksInDegrees, MAX_ARM_POWER, ARM_INITIAL_ANGLE);

        // Set directions for drivetrain motors
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        waitForStart();

        armController.setTargetPosition(ARM_UP_POSITION);
        forward(DRIVETRAIN_POWER);
        turnLeft(DRIVETRAIN_POWER);
        stopDriving();
        outtakePreload();
    }


    //functions for driving, can be edited
    public void forward(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);

    }

    public void backward(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(-power);
    }

    public void turnLeft(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void turnRight(double power) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    public void stopDriving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void outtakePreload(){
        armController.setTargetPosition(OUTTAKE_POS);
        intakeServo.setPower(OUTTAKE_POWER);
    }
}
