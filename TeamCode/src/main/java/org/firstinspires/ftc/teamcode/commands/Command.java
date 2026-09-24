package org.firstinspires.ftc.teamcode.commands;

/**
 * Base interface for robot commands in a command-based architecture.
 */
public interface Command {
    /**
     * Called once when the command is initialized.
     */
    default void initialize() {}

    /**
     * Called repeatedly while the command is active.
     */
    default void execute() {}

    /**
     * Called once when the command ends or is interrupted.
     *
     * @param interrupted whether the command was interrupted or finished naturally
     */
    default void end(boolean interrupted) {}

    /**
     * Indicates whether the command has finished executing.
     *
     * @return true if finished, false otherwise
     */
    default boolean isFinished() {
        return false;
    }
}
