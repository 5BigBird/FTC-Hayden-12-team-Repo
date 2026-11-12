package org.firstinspires.ftc.teamcode.automode;

import org.firstinspires.ftc.teamcode.automode.MoveRobot.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Luc auto webcam test", group="Auto")

public class AutoWebcam extends LinearOpMode {
    final double DESIRED_DISTANCE = 30.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;    //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;    //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;    //  Clip the turn speed to this max value (adjust for your robot)
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private static final boolean USE_WEBCAM = true;
    private final int DESIRED_TAG_ID = -1;    // The tag you want to approach, or set to -1 for ANY tag.
    private final String DESIRED_CLUSTER_NAME = null; // The cluster name you want to approach, or set null for ANY cluster.
    private final double PI = 3.1415926535;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private boolean targetFound  = false;    // Set to true when an AprilTag/Cluster target is detected
    private String targetName    = "none";
    private int    targetID      = 0;
    private double targetRange   = 0;
    private double targetBearing = 0;
    private double targetYaw     = 0;
    @Override public void runOpMode() {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  angle          = 0;        // Desired angle to turn power/speed (-1 to +1)

        initAprilTag();

        telemetryAprilTag();
        // Display live camera performance diagnostics on Driver Station telemetry
        telemetry.addData("Status", "Streaming Live");
        telemetry.addData("Frames Per Second (FPS)", "%.1f", visionPortal.getFps());
        telemetry.addData("Camera Connection State", visionPortal.getCameraState());
        telemetry.update(); // Send telemetry packet to Driver Station

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRight");


        // 1. Fix motor directions (Reverse LEFT side, Forward RIGHT side)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

// IMPORTANT: Make sure MoveRobot is initialized with your hardwareMap or motors!
// Example: MoveRobot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                    if (singleDetection.metadata != null && ((DESIRED_TAG_ID < 0) || (singleDetection.id == DESIRED_TAG_ID))) {
                        targetName    = singleDetection.metadata.name;
                        targetID      = singleDetection.id;
                        targetRange   = singleDetection.ftcPose.range;
                        targetBearing = singleDetection.ftcPose.bearing;
                        targetYaw     = singleDetection.ftcPose.yaw;
                        targetFound   = true;
                        break;
                    }
                } else if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    if (DESIRED_CLUSTER_NAME == null || clusterDet.metadata.shortName.equals(DESIRED_CLUSTER_NAME)) {
                        targetName    = clusterDet.metadata.shortName;
                        targetID      = -1;
                        targetRange   = clusterDet.ftcPose.range;
                        targetBearing = clusterDet.ftcPose.bearing;
                        targetYaw     = clusterDet.ftcPose.yaw;
                        targetFound   = true;
                        break;
                    }
                }
            }

            if (targetFound) {
                double rangeError = targetRange - DESIRED_DISTANCE;

                // Check if the robot has reached the target (within 1.5 inches and 2 degrees)
                if (Math.abs(rangeError) < 1.5 && Math.abs(targetBearing) < 2.0) {
                    MoveRobot.driveAngular(0, 0);
                    telemetry.addData("Status", "Target Reached!");
                    telemetry.update();
                    break; // Exit the autonomous loop once parked at the tag
                }

                // Calculate proportional drive and turn powers
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                // Note: If MoveRobot.driveAngular expects raw degrees, keep `angle = targetBearing;`
                // If it expects motor turn power (-1 to 1), use the line below:
                angle = Range.clip(targetBearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Found", "ID %d (%s)", targetID, targetName);
                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, angle);
                MoveRobot.driveAngular(drive, angle);
            } else {
                // Stop safely (or set a slow turn speed like 0.15 to search for a tag)
                telemetry.addData("Status", "Searching for AprilTag...");
                MoveRobot.driveAngular(0, 0);
            }

            telemetry.update();
            sleep(10);
        }

// Ensure motors are completely stopped when OpMode finishes
        MoveRobot.driveAngular(0, 0);

    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}
