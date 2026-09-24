package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Robot-centric TeleOp OpMode using DriveSubsystem and DriveCommand.
 * Designed for Logitech F310 Gamepad (or Xbox / PS Controllers):
 * - Left Stick Y: Forward / Reverse
 * - Left Stick X: Strafe Left / Right
 * - Right Stick X: Turn Left / Right
 * - Left Bumper (Hold): Slow Mode (30% speed)
 * - Right Bumper (Hold): Medium Mode (60% speed)
 * - Default: Full Speed (100% speed)
 */
@TeleOp(name = "Robot Centric TeleOp", group = "TeleOp")
public class RobotTeleOp extends LinearOpMode {

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    @Override
    public void runOpMode() {
        // Initialize subsystem with FTC HardwareMap
        driveSubsystem.init(hardwareMap);

        // Bind Logitech F310 controller inputs to DriveCommand
        // Note: gamepad stick Y is inverted on controllers (pushing forward is negative)
        DriveCommand driveCommand = new DriveCommand(
                driveSubsystem,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x,
                () -> gamepad1.left_bumper ? 0.3 : (gamepad1.right_bumper ? 0.6 : 1.0),
                0.05
        );

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Mode", "Robot-Centric");
        telemetry.addData("Controller", "Logitech F310");
        telemetry.update();

        // Wait for the driver to press PLAY
        waitForStart();

        driveCommand.initialize();

        // Main execution loop
        while (opModeIsActive() && !isStopRequested()) {
            driveCommand.execute();

            // Telemetry updates for debugging wheel powers
            telemetry.addData("Status", "Running");
            telemetry.addData("FL Power", "%.2f", driveSubsystem.getFrontLeft().getPower());
            telemetry.addData("FR Power", "%.2f", driveSubsystem.getFrontRight().getPower());
            telemetry.addData("BL Power", "%.2f", driveSubsystem.getBackLeft().getPower());
            telemetry.addData("BR Power", "%.2f", driveSubsystem.getBackRight().getPower());
            telemetry.update();
        }

        // Clean up when OpMode stops
        driveCommand.end(false);
    }
}
