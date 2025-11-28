package org.firstinspires.ftc.teamcode.commandbase.subsystems;


import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

import com.qualcomm.robotcore.hardware.DcMotor;

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

public class Shooter implements Subsystem {

    public final static Shooter INSTANCE = new Shooter();

    private Shooter() { }


    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").brakeMode().zeroed();
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").brakeMode().zeroed();


    ControlSystem controller = ControlSystem.builder()
            .velPid(shooterP, shooterI, shooterD)
            .basicFF(shooterFF)
            .build();

    @Override
    public void initialize() {
        shooterMotorRight.zeroed();
        shooterMotorLeft.zeroed();
        controller.setGoal(new KineticState(0,0,0));
    }

    public Command shooterOn() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, targetVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterClassifier() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, classifierVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterAutoClassifier() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, classifierAutoVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterReverse() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, -2000, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterOff() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, shooterOffVelocity, 0)))
                .setIsDone(() -> true);
    }
    public Command waitUntilShooterAtTargetVelocity(double tolerance, double targetVel, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new WaitUntil(() ->
                (shooterMotorRight.getVelocity() - targetVel) <= tolerance
        ).then(command);
    }




    @Override
    public void periodic() {

        shooterMotorLeft.setPower(-controller.calculate(shooterMotorLeft.getState()));
        shooterMotorRight.setPower(controller.calculate(shooterMotorRight.getState()));

        ActiveOpMode.telemetry().addData("Right Shooter Motor Velocity:", shooterMotorRight.getVelocity());
        ActiveOpMode.telemetry().addData("Left Shooter Motor Velocity:", shooterMotorLeft.getVelocity());
    }
}

