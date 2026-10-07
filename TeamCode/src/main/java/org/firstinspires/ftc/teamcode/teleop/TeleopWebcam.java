package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Webcam", group = "TeleOp")
public class TeleopWebcam extends LinearOpMode {

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize the webcam named "Webcam 1" in your Robot Configuration
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam1"))
                .build();

        telemetry.addData("Camera", "Initialized");
        telemetry.addData("View Stream", "Press 3 dots menu in Driver Station -> Camera Stream");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Streaming");
            telemetry.addData("FPS", "%.1f", visionPortal.getFps());
            telemetry.addData("State", visionPortal.getCameraState());
            telemetry.update();
        }

        // Close the portal when stopping
        visionPortal.close();
    }
}
