package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {

    public final static Shooter INSTANCE = new Shooter();

    private Shooter() { }

    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode();
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").reversed().brakeMode();

    MotorGroup shooterMotorGroup = new MotorGroup(shooterMotorLeft, shooterMotorRight); //create motor group

    private final PIDCoefficients shooterPIDComponents = new PIDCoefficients(shooterP, shooterI, shooterD);

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(shooterPIDComponents)
            .build();

    public Command shooterOn = new RunToVelocity(shooterController, targetVelocity).requires(this);
    public Command shooterOff = new RunToVelocity(shooterController, shooterOffVelocity).requires(this);

    @Override
    public void periodic() {
        shooterMotorGroup.setPower(shooterController.calculate(shooterMotorGroup.getState()));
        ActiveOpMode.telemetry().addData("Right Shooter Motor Velocity:", shooterMotorRight.getVelocity());
        ActiveOpMode.telemetry().addData("Left Shooter Motor Velocity:", shooterMotorLeft.getVelocity());
        ActiveOpMode.telemetry().addData("Motor Group Velocity", shooterMotorGroup.getVelocity());
    }
}
