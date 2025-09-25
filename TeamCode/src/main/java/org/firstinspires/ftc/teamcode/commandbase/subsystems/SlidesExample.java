package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class SlidesExample implements Subsystem {

    public static final SlidesExample INSTANCE = new SlidesExample();

    private SlidesExample() { }

    private MotorEx slidesMotor = new MotorEx("slidesMotor");

    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(1,1,1)
            .elevatorFF(0)
            .build();

    public Command toLow = new RunToPosition(controlSystem, 0).requires(this);

    public Command toMiddle = new RunToPosition(controlSystem, 500).requires(this);

    public Command toHigh = new RunToPosition(controlSystem, 1200).requires(this);

    @Override
    public void periodic() {
        slidesMotor.setPower(controlSystem.calculate(slidesMotor.getState()));
    }
}