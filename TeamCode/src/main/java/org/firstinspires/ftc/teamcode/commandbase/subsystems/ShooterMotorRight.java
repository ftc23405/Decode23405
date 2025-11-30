package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterMotorRight implements Subsystem {

    public final static ShooterMotorRight INSTANCE = new ShooterMotorRight();

    private ShooterMotorRight() { }
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").brakeMode().zeroed();

    private static double aveRPM = 0;


    ControlSystem controllerRight = ControlSystem.builder()
            .velPid(shooterP, shooterI, shooterD)
            .basicFF(shooterFF)
            .build();


    @Override
    public void initialize() {
        shooterMotorRight.zeroed();
        controllerRight.setGoal(new KineticState(0,0,0));
    }

    public Command shooterMotorRightOn() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, targetVelocity, 0)))
                .setIsDone(() -> controllerRight.isWithinTolerance(new KineticState(0,0)));
    }

    public Command shooterMotorRightClassifier() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, classifierVelocity, 0)))
                .setIsDone(() -> controllerRight.isWithinTolerance(new KineticState(0,0)));
    }

    public Command shooterMotorAutoRightClassifier() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, classifierAutoVelocity, 0)))
                .setIsDone(() -> controllerRight.isWithinTolerance(new KineticState(0,0)));
    }

    public Command shooterMotorRightReverse() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, -2000, 0)))
                .setIsDone(() -> controllerRight.isWithinTolerance(new KineticState(0,0)));
    }

    public Command shooterMotorRightOff() {
        return new LambdaCommand()
                .setStart(() -> controllerRight.setGoal(new KineticState(0, shooterOffVelocity, 0)))
                .setIsDone(() -> controllerRight.isWithinTolerance(new KineticState(0,0)));
    }
    public Command waitUntilShooterRightAtTargetVelocity(double tolerance, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new IfElseCommand(() ->
                (Math.abs((calculateRPM(shooterMotorRight.getVelocity(), 28) - controllerRight.getGoal().getVelocity())) <= tolerance), command);
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(aveRPM - controllerRight.getGoal().getVelocity()) <= tolerance;
    }

    @Override
    public void periodic() {
        double motorRPM = calculateRPM(shooterMotorRight.getVelocity(), 28);
        int avePeriod = 5;
        aveRPM = aveRPM * (avePeriod - 1) / avePeriod + motorRPM / avePeriod;
        shooterMotorRight.setPower(controllerRight.calculate(new KineticState(0, aveRPM)));
        ActiveOpMode.telemetry().addData("Right Shooter Motor Velocity:", aveRPM);
        ActiveOpMode.telemetry().addData("Right Controller Velocity:", controllerRight.getGoal().getVelocity());
    }
}
