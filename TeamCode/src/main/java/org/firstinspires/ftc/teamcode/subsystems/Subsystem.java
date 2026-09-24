package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Base interface for robot subsystems in a command-based architecture.
 */
public interface Subsystem {
    /**
     * Called periodically during the OpMode loop execution.
     */
    default void periodic() {}
}
