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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Luc auto v1 test", group="Auto")
public class AutoShoot extends LinearOpMode {

    // Hardware Declarations
    private DcMotorEx flywheelMotor;
    private Servo feederServo; // Feeder/kicker pushing pollen into the flywheel

    // Target Velocity (Ticks per second) - Adjust based on your shooting distance
    private final double TARGET_VELOCITY = 1200.0;
    // Tolerance window (Allow shooting if flywheel is within 50 ticks of target)
    private final double VELOCITY_TOLERANCE = 50.0;

    // Servo positions for feeding pollen
    private final double FEEDER_REST_POS = 0.0;
    private final double FEEDER_PUSH_POS = 1.0;

    @Override
    public void runOpMode() {
        // 1. Initialization
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel_motor");
        feederServo = hardwareMap.get(Servo.class, "feeder_servo");

        // Essential: Reset encoders and set to run using velocity control
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Optional: Lift standard 85% speed cap restriction if extra juice is needed

        feederServo.setPosition(FEEDER_REST_POS);

        telemetry.addData("Status", "Initialized. Waiting for Start...");
        telemetry.update();

        waitForStart();

        // 2. Main TeleOp Loop
        while (opModeIsActive()) {
            double currentVelocity = flywheelMotor.getVelocity();

            // Right Bumper = Auto Launch Sequence
            if (gamepad1.right_bumper) {
                // Spin up flywheel to precise target velocity
                flywheelMotor.setVelocity(TARGET_VELOCITY);

                // Auto-sequence: Only feed pollen if flywheel speed is within tolerance
                if (currentVelocity >= (TARGET_VELOCITY - VELOCITY_TOLERANCE)) {
                    feederServo.setPosition(FEEDER_PUSH_POS);
                } else {
                    // Stay at rest while spinning up
                    feederServo.setPosition(FEEDER_REST_POS);
                }
            }
            // Manual Flywheel Override (e.g., holding Right Trigger to rev up early)
            else if (gamepad1.right_trigger > 0.1) {
                flywheelMotor.setVelocity(TARGET_VELOCITY * gamepad1.right_trigger);
                feederServo.setPosition(FEEDER_REST_POS);
            }
            // Turn everything off when no shooting buttons are pressed
            else {
                flywheelMotor.setVelocity(0);
                feederServo.setPosition(FEEDER_REST_POS);
            }

            // 3. Telemetry Feedback for Driver Hub tuning
            telemetry.addData("Target Velocity", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Velocity Error", TARGET_VELOCITY - currentVelocity);
            telemetry.addData("Ready to Shoot?", (currentVelocity >= (TARGET_VELOCITY - VELOCITY_TOLERANCE)));
            telemetry.update();
        }
    }
}