package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {

    public final static Shooter INSTANCE = new Shooter();

    private Shooter() { }

    public static double shooterP = 0.01;
    public static double shooterI = 0.05; //use integrator (high kI) for high error response
    public static double shooterD = 0;

    public static double targetVelocity = 500;
    public static double offVelocity = 0;

    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode();
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").reversed().brakeMode();

    MotorGroup shooterMotorGroup = new MotorGroup(shooterMotorLeft, shooterMotorRight); //create motor group

    private final PIDCoefficients shooterPIDComponents = new PIDCoefficients(shooterP, shooterI, shooterD);

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(shooterPIDComponents)
            .build();

    public Command shooterOn = new RunToVelocity(shooterController, targetVelocity).requires(this);

    public Command shooterOff = new RunToVelocity(shooterController, offVelocity).requires(this);

    @Override
    public void periodic() {
        shooterMotorGroup.setPower(shooterController.calculate(shooterMotorGroup.getState()));
    }
}
