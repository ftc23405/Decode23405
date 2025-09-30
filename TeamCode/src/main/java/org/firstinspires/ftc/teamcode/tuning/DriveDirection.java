package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.text.DecimalFormat;
import java.util.zip.Adler32;

@Configurable
@TeleOp
public class DriveDirection extends OpMode {

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    private DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    @Override
    public void init() {
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {

        if (gamepad1.a){
            frontLeftMotor.setPower(1);
            telemetryM.debug("frontLeftMotor running: ", frontLeftMotor.getPower());
        }
        if (gamepad1.b){
            backLeftMotor.setPower(1);
            telemetryM.debug("backLeftMotor running: ", backLeftMotor.getPower());
        }
        if (gamepad1.x){
            frontRightMotor.setPower(1);
            telemetryM.debug("frontRightMotor running: ", frontRightMotor.getPower());
        }
        if (gamepad1.y){
            backRightMotor.setPower(1);
            telemetryM.debug("backRightMotor running: ", backRightMotor.getPower());
        }
        if (gamepad1.right_bumper){ //stops all motors
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }


        telemetryM.update(telemetry);
    }
}

