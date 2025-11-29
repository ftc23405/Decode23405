package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class ServoTester extends LinearOpMode {
    public static double WRIST_SERVO_POSITION = 0;
    public void runOpMode() throws InterruptedException {
        final Servo transferPusher;
        transferPusher = hardwareMap.get(Servo.class, "transferPusher");
        waitForStart();

        while(opModeIsActive()){
            transferPusher.setPosition(WRIST_SERVO_POSITION);
        }
    }

}
