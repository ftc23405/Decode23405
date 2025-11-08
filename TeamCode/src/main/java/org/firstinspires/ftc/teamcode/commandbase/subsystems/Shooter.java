package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {

    public final static Shooter INSTANCE = new Shooter();

    private Shooter() { }

    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").reversed().floatMode();
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").floatMode();

    MotorGroup shooterMotorGroup = new MotorGroup(shooterMotorLeft, shooterMotorRight); //create motor group


    public PIDCoefficients shooterPIDComponents = new PIDCoefficients(shooterP, shooterI, shooterD);

    public final ControlSystem shooterController = ControlSystem.builder()
            .velPid(shooterPIDComponents)
            .basicFF(shooterFF)
            .build();

    public Command shooterFarShoot() {
        return new LambdaCommand()
                .setStart(()-> shooterController.setGoal(new KineticState(0, targetVelocity,0)))
                .setIsDone(() -> true);
    }
    public Command shooterCloseShoot() {
        return new LambdaCommand()
                .setStart(() -> shooterController.setGoal(new KineticState(0,targetVelocity / 2, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterClassifierShoot() {
        return new LambdaCommand()
                .setStart(() -> shooterController.setGoal(new KineticState(0,targetVelocity / 4, 0)))
                .setIsDone(() -> true);
    }
    public Command shooterOff() {
        return new LambdaCommand()
                .setStart(() -> shooterController.setGoal(new KineticState(0, shooterOffVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command waitUntilAtTargetVelocity(double tolerance, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new WaitUntil(() ->
                Math.abs(shooterMotorGroup.getVelocity() - targetVelocity) < tolerance
        ).then(command);
    }

    @Override
    public void periodic() {
        shooterMotorGroup.setPower(shooterController.calculate(shooterMotorGroup.getState()));
        ActiveOpMode.telemetry().addData("Right Shooter Motor Velocity:", shooterMotorRight.getVelocity());
        ActiveOpMode.telemetry().addData("Left Shooter Motor Velocity:", shooterMotorLeft.getVelocity());
        ActiveOpMode.telemetry().addData("Motor Group Velocity", shooterMotorGroup.getVelocity());
    }
}
