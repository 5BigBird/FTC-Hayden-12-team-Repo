package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveCommand implements Command {

    private final DriveSubsystem drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;

    public DriveCommand(DriveSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
    }

    @Override
    public void execute() {
        drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
    }

    @Override
    public void end() {
        drive.stop();
    }
}
