package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandbase.subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.HolonomicMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;

@TeleOp
@Configurable //panels
public class V1_Teleop extends NextFTCOpMode {

    public V1_Teleop() {
        addComponents( //add needed components
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    //not in use (using pedro teleop), create motors and set directions
    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor");
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").reversed();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor");
    private final IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed();


    @Override
    public void onInit() {

    }


    @Override
    public void onStartButtonPressed() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
        driverControlled.schedule();

        Gamepads.gamepad1().a()
                .whenBecomesTrue(Intake.INSTANCE.intakeFullSpeed);
        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.intakeOff);
    }

    @Override
    public void onUpdate() {
        ActiveOpMode.telemetry();
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
