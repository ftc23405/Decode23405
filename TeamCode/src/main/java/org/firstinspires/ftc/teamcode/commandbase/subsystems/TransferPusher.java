package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import static org.firstinspires.ftc.teamcode.tuning.Globals.*;

public class TransferPusher implements Subsystem {

    private TransferPusher() {}

    public final static TransferPusher INSTANCE = new TransferPusher();

    private final CRServoEx transferPusher = new CRServoEx("transferPusher");

    public Command transferOn = new SetPower(transferPusher, transferPower).requires(this);

    public Command transferReverse = new SetPower(transferPusher, reverseTransferPower).requires(this);

    public Command transferSlowReverse = new SetPower(transferPusher, reverseTransferPower / 5).requires(this);

    public Command transferOff = new SetPower(transferPusher, 0).requires(this);

    @Override
    public void initialize() {
    }

    @Override
    public void periodic() {

    }
}
