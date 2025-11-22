package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class ShooterMotorLeft implements Subsystem {

    public final static ShooterMotorLeft INSTANCE = new ShooterMotorLeft();

    private ShooterMotorLeft() { }

    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode().zeroed();

    ControlSystem controllerLeft = ControlSystem.builder()
            .velPid(shooterP, shooterI, shooterD)
            .basicFF(shooterFF)
            .build();


    @Override
    public void initialize() {
        shooterMotorLeft.zeroed();
        controllerLeft.setGoal(new KineticState(0,0,0));
    }

    public Command shooterMotorLeftOn() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -targetVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterMotorLeftClassifier() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -classifierVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterMotorAutoLeftClassifier() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -classifierAutoVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterMotorLeftOff() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -shooterOffVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterMotorLeftReverse() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, 2000, 0)))
                .setIsDone(() -> true);
    }
    public Command waitUntilShooterLeftAtTargetVelocity(double tolerance, double targetVel, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new WaitUntil(() ->
                Math.abs(shooterMotorLeft.getVelocity() - targetVel) < tolerance
        ).then(command);
    }

    @Override
    public void periodic() {
        shooterMotorLeft.setPower(controllerLeft.calculate(shooterMotorLeft.getState()));
        ActiveOpMode.telemetry().addData("Left Shooter Motor Velocity:", shooterMotorLeft.getVelocity());
    }
}
