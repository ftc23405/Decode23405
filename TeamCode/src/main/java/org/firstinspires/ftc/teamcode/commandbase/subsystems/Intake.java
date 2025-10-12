package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Intake implements Subsystem {

    public static double intakeP = 1;
    public static double intakeI = 0;
    public static double intakeD = 0.005;

    public static double intakeSpeed = 1000;
    public static double offSpeed = 0;

    public final static Intake INSTANCE = new Intake();

    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intakeMotor").brakeMode();

    private final PIDCoefficients intakePIDComponents = new PIDCoefficients(intakeP, intakeI, intakeD);

    private final ControlSystem intakeController = ControlSystem.builder()
            .velPid(intakePIDComponents)
            .build();

    public Command intakeOff = new RunToVelocity(intakeController, offSpeed).requires(this);

    public Command intakeHalfSpeed = new RunToVelocity(intakeController, intakeSpeed / 2).requires(this);

    public Command intakeFullSpeed = new RunToVelocity(intakeController, intakeSpeed).requires(this);

    public Command intakeReverseHalfSpeed = new RunToVelocity(intakeController, -intakeSpeed / 2).requires(this);

    public Command intakeReverseFullSpeed = new RunToVelocity(intakeController, -intakeSpeed).requires(this);

    @Override
    public void periodic() {
        intakeMotor.setPower(intakeController.calculate(intakeMotor.getState()));
        ActiveOpMode.telemetry().addData("Intake Velocity:", intakeMotor.getVelocity());
        ActiveOpMode.telemetry().update();
    }
}
