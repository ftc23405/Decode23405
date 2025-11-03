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
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Intake implements Subsystem {

    public final static Intake INSTANCE = new Intake();

    private Intake() { }

    private final MotorEx intakeMotor = new MotorEx("intakeMotor").reversed().brakeMode();

    public Command intakeOff = new SetPower(intakeMotor, intakeOffSpeed);

    public Command intakeQuarterSpeed = new SetPower(intakeMotor, intakeTargetSpeed / 4);

    public Command intakeHalfSpeed = new SetPower(intakeMotor, intakeTargetSpeed / 2);

    public Command intakeFullSpeed = new SetPower(intakeMotor, intakeTargetSpeed);

    public Command intakeReverseHalfSpeed = new SetPower(intakeMotor, -intakeTargetSpeed / 2);
    public Command intakeReverseSlow = new SetPower(intakeMotor, -0.2 * intakeTargetSpeed);
    public Command intakeReverseFullSpeed = new SetPower(intakeMotor, -intakeTargetSpeed);


//    public Command variableIntake(DoubleSupplier triggerSupplier) {
//        DoubleSupplier velocitySupplier = () -> {
//            double raw = triggerSupplier.getAsDouble(); // assume 0..1
//            // deadzone:
//            double deadzone = 0.05;
//            if (Math.abs(raw) < deadzone) return 0.0;
//            // optional squared scaling
//            double scaled = Math.signum(raw) * (raw * raw);
//            return scaled * intakeTargetSpeed;
//        };
//        return new RunToVelocity(intakeController, velocitySupplier.getAsDouble()).requires(this);
//    }

    @Override
    public void periodic() {
        ActiveOpMode.telemetry().addData("Intake Velocity", intakeMotor.getVelocity());
    }
}
