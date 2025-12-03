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

public class ShooterMotorLeft implements Subsystem {

    public final static ShooterMotorLeft INSTANCE = new ShooterMotorLeft();

    private ShooterMotorLeft() { }

    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode().zeroed();

    ControlSystem controllerLeft = ControlSystem.builder()
            .velPid(shooterP, shooterI, shooterD)
            .basicFF(shooterFF)
            .build();

    private static double aveRPM = 0;


    @Override
    public void initialize() {
        shooterMotorLeft.zeroed();
        controllerLeft.setGoal(new KineticState(0,0,0));
    }

    public Command shooterMotorLeftFar() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -targetVelocity, 0)));
    }

    public Command shooterMotorLeftClassifier() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -classifierVelocity, 0)));
    }

    public Command shooterMotorAutoLeftClassifier() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -classifierAutoVelocity, 0)));
    }

    public Command shooterMotorLeftOff() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -shooterOffVelocity, 0)));
    }

    public Command autoRPM(double rpm) {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, -rpm, 0)));
    }

    public Command shooterMotorLeftReverse() {
        return new LambdaCommand()
                .setStart(() -> controllerLeft.setGoal(new KineticState(0, 2000, 0)));
    }
    public Command waitUntilShooterLeftAtTargetVelocity(double tolerance, double targetVel, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new WaitUntil(() ->
                Math.abs(shooterMotorLeft.getVelocity() - targetVel) < tolerance
        ).then(command);
    }

    @Override
    public void periodic() {
        double motorRPM = calculateRPM(shooterMotorLeft.getVelocity(), 28);
        int avePeriod = 5;
        aveRPM = aveRPM * (avePeriod - 1) / avePeriod + motorRPM / avePeriod;
        shooterMotorLeft.setPower(controllerLeft.calculate(new KineticState(0, aveRPM)));
        ActiveOpMode.telemetry().addData("Left Shooter Motor Velocity:", aveRPM);
    }
}
