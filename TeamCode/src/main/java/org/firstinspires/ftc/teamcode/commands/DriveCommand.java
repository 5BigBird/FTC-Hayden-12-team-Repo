package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * DriveCommand reads controller inputs (such as Logitech F310 joysticks)
 * and drives the robot in a robot-centric frame with deadband filtering and speed scaling.
 */
public class DriveCommand implements Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;
    private final DoubleSupplier speedMultiplierSupplier;
    private final double deadband;

    /**
     * Constructs a DriveCommand with default speed multiplier (1.0) and deadband (0.05).
     *
     * @param driveSubsystem  the DriveSubsystem to command
     * @param forwardSupplier supplier for forward/axial motion (+forward)
     * @param strafeSupplier  supplier for strafe/lateral motion (+right)
     * @param turnSupplier    supplier for turn/yaw motion (+clockwise)
     */
    public DriveCommand(DriveSubsystem driveSubsystem,
                        DoubleSupplier forwardSupplier,
                        DoubleSupplier strafeSupplier,
                        DoubleSupplier turnSupplier) {
        this(driveSubsystem, forwardSupplier, strafeSupplier, turnSupplier, () -> 1.0, 0.05);
    }

    /**
     * Constructs a DriveCommand with a custom speed multiplier supplier and deadband threshold.
     *
     * @param driveSubsystem          the DriveSubsystem to command
     * @param forwardSupplier         supplier for forward/axial motion (+forward)
     * @param strafeSupplier          supplier for strafe/lateral motion (+right)
     * @param turnSupplier            supplier for turn/yaw motion (+clockwise)
     * @param speedMultiplierSupplier supplier for dynamic speed scaling (0.0 to 1.0)
     * @param deadband                input deadband threshold (0.0 to 1.0)
     */
    public DriveCommand(DriveSubsystem driveSubsystem,
                        DoubleSupplier forwardSupplier,
                        DoubleSupplier strafeSupplier,
                        DoubleSupplier turnSupplier,
                        DoubleSupplier speedMultiplierSupplier,
                        double deadband) {
        this.driveSubsystem = driveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.turnSupplier = turnSupplier;
        this.speedMultiplierSupplier = speedMultiplierSupplier;
        this.deadband = deadband;
    }

    @Override
    public void execute() {
        double multiplier = speedMultiplierSupplier != null ? speedMultiplierSupplier.getAsDouble() : 1.0;

        double forward = applyDeadband(forwardSupplier.getAsDouble(), deadband) * multiplier;
        double strafe = applyDeadband(strafeSupplier.getAsDouble(), deadband) * multiplier;
        double turn = applyDeadband(turnSupplier.getAsDouble(), deadband) * multiplier;

        driveSubsystem.driveRobotCentric(forward, strafe, turn);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }
}
