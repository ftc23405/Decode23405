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

    MotorEx shooterMotorLeft = new MotorEx("shooterMotorLeft").reversed().floatMode();
    MotorEx shooterMotorRight = new MotorEx("shooterMotorRight").floatMode();

    MotorGroup shooterMotorGroup = new MotorGroup(shooterMotorLeft, shooterMotorRight); //create motor group

    ControlSystem controller = ControlSystem.builder()
            .velPid(0.011, 0, 0)
            .basicFF(0.0005)
            .build();


    @Override
    public void initialize() {
        shooterMotorLeft.zeroed();
        shooterMotorRight.zeroed();
        controller.setGoal(new KineticState(0,0,0));
    }

    public Command shooterToTargetVelocity() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, targetVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterNewOff() {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, shooterOffVelocity, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterFarShoot = new SetPower(shooterMotorGroup, 1.1);

    public Command shooterClassifierShoot = new SetPower(shooterMotorGroup, 0.8);

    public Command shooterCloseShoot = new SetPower(shooterMotorGroup, 0.7);

    public Command shooterOff = new SetPower(shooterMotorGroup, 0);
    public Command waitUntilAtTargetVelocity(double tolerance, Command command) { //waits until shooter is at target velocity with inputed tolerance, then runs the command passed as an argument
        return new WaitUntil(() ->
                Math.abs(shooterMotorGroup.getVelocity() - targetVelocity) < tolerance
        ).then(command);
    }

    @Override
    public void periodic() {
        shooterMotorGroup.setPower(controller.calculate(shooterMotorGroup.getState()));
        ActiveOpMode.telemetry().addData("Right Shooter Motor Velocity:", shooterMotorRight.getVelocity());
        ActiveOpMode.telemetry().addData("Left Shooter Motor Velocity:", shooterMotorLeft.getVelocity());
        ActiveOpMode.telemetry().addData("Motor Group Velocity", shooterMotorGroup.getVelocity());
    }
}
