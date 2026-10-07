package org.firstinspires.ftc.teamcode.teleop;

// Import FTC SDK classes required for OpModes and Camera Vision Portal
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Simple TeleOp OpMode that initializes a USB webcam and streams video footage.
 * To view the live camera stream on your Driver Station app:
 * 1. Select and INIT this "Webcam" TeleOp.
 * 2. Tap the 3 dots menu in the upper-right corner of the Driver Station screen.
 * 3. Select "Camera Stream".
 */
@TeleOp(name = "Webcam", group = "TeleOp") // Registers OpMode on Driver Station under TeleOp
public class TeleopWebcam extends LinearOpMode {

    // Member variable to manage the camera stream lifecycle
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Step 1: Retrieve webcam hardware configuration ("Webcam 1") from the Robot Controller
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Step 2: Build and start the VisionPortal instance for live camera streaming
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName) // Connect the webcam hardware instance
                .build();              // Construct and open the vision portal

        // Step 3: Send initialization status and driver instructions to telemetry screen
        telemetry.addData("Camera Status", "Initialized and Streaming");
        telemetry.addData("How to View Stream", "Tap 3-dots menu on Driver Station -> Camera Stream");
        telemetry.update();

        // Step 4: Pause code execution here until driver presses the PLAY button on Driver Station
        waitForStart();

        // Step 5: Main execution loop while the match/OpMode is running
        while (opModeIsActive()) {
            // Display live camera performance diagnostics on Driver Station telemetry
            telemetry.addData("Status", "Streaming Live");
            telemetry.addData("Frames Per Second (FPS)", "%.1f", visionPortal.getFps());
            telemetry.addData("Camera Connection State", visionPortal.getCameraState());
            telemetry.update(); // Send telemetry packet to Driver Station
        }

        // Step 6: Properly release camera hardware resources when the OpMode stops or finishes
        visionPortal.close();
    }
}
