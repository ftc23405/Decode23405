package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorLeft;
import org.firstinspires.ftc.teamcode.commandbase.subsystems.ShooterMotorRight;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
@Configurable
@Config
public class VelocityPIDTuner extends NextFTCOpMode {

    public VelocityPIDTuner() {
        addComponents(
                new SubsystemComponent(ShooterMotorRight.INSTANCE, ShooterMotorLeft.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;

    public static double ff = 0;
    public static int targetRPM = 0;

    ControlSystem controller = ControlSystem.builder()
            .velPid(p, i, d)
            .basicFF(ff)
            .build();

    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").brakeMode().zeroed();
    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode().zeroed();




    @Override
    public void onInit() {



        controller.setGoal(new KineticState(0, 0 ,0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry(), PanelsTelemetry.INSTANCE.getFtcTelemetry());
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> controller.setGoal(new KineticState(0, calculateTicksPerSecond(targetRPM, 28) ,0)));
    }

    @Override
    public void onUpdate() {

        shooterMotorRight.setPower(controller.calculate(shooterMotorRight.getState()));
        shooterMotorLeft.setPower(controller.calculate(shooterMotorRight.getState()));

        telemetry.addData("Shooter Left Velocity", shooterMotorLeft.getVelocity());
        telemetry.addData("Shooter Right Velocity", shooterMotorRight.getVelocity());
        telemetry.update();
    }
}
