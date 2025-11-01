package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import com.bylazar.configurables.annotations.Configurable;

import java.util.function.DoubleSupplier;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.GamepadEx;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Intake implements Subsystem {

    public final static Intake INSTANCE = new Intake();

    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intakeMotor").brakeMode().reversed();

    private final PIDCoefficients intakePIDComponents = new PIDCoefficients(intakeP, intakeI, intakeD);

    private final ControlSystem intakeController = ControlSystem.builder()
            .velPid(intakePIDComponents)
            .build();

    public Command intakeOff = new RunToVelocity(intakeController, intakeOffSpeed).requires(this);

    public Command intakeHalfSpeed = new RunToVelocity(intakeController, intakeTargetSpeed / 2).requires(this);

    public Command intakeFullSpeed = new RunToVelocity(intakeController, intakeTargetSpeed).requires(this);

    public Command intakeReverseHalfSpeed = new RunToVelocity(intakeController, -intakeTargetSpeed / 2).requires(this);

    public Command intakeReverseSlow = new RunToVelocity(intakeController, -0.1 * intakeTargetSpeed);
    public Command intakeReverseFullSpeed = new RunToVelocity(intakeController, -intakeTargetSpeed).requires(this);


    public Command variableIntake(DoubleSupplier triggerSupplier) {
        DoubleSupplier velocitySupplier = () -> {
            double raw = triggerSupplier.getAsDouble(); // assume 0..1
            // deadzone:
            double deadzone = 0.05;
            if (Math.abs(raw) < deadzone) return 0.0;
            // optional squared scaling
            double scaled = Math.signum(raw) * (raw * raw);
            return scaled * intakeTargetSpeed;
        };
        return new RunToVelocity(intakeController, velocitySupplier.getAsDouble()).requires(this);
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(intakeController.calculate(intakeMotor.getState()));
        ActiveOpMode.telemetry().addData("Intake Velocity", intakeMotor.getVelocity());
    }
}
