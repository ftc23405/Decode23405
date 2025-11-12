package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterMotorRight implements Subsystem {

    public final static ShooterMotorRight INSTANCE = new ShooterMotorRight();

    private ShooterMotorRight() { }
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").floatMode().zeroed();

    ControlSystem controllerRight = ControlSystem.builder()
            .velPid(shooterP, shooterI, shooterD)
            .basicFF(shooterFF)
            .build();


    @Override
    public void initialize() {
        controllerRight.setGoal(new KineticState(0,0,0));
    }

    public Command shooterMotorRightOn() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, targetVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterMotorRightOff() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, shooterOffVelocity, 0)))
                .setIsDone(() -> true);
    }
    public Command waitUntilShooterRightAtTargetVelocity(double tolerance, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new WaitUntil(() ->
                Math.abs(shooterMotorRight.getVelocity() - targetVelocity) < tolerance
        ).then(command);
    }

    @Override
    public void periodic() {
        shooterMotorRight.setPower(controllerRight.calculate(shooterMotorRight.getState()));
        ActiveOpMode.telemetry().addData("Right Shooter Motor Velocity:", shooterMotorRight.getVelocity());
    }
}
