package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

public class TransferPusher implements Subsystem {

    private TransferPusher() {}

    public final static TransferPusher INSTANCE = new TransferPusher();

    private final ServoEx transferPusher = new ServoEx("transferPusher");

    public Command transferPush = new SetPosition(transferPusher, transferPushPosition).requires(this);

    public Command transferHold = new SetPosition(transferPusher, transferHoldPosition).requires(this);

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

    }
}
