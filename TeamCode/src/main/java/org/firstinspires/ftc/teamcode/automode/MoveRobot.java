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

@Autonomous(name="Luc auto move v1 test", group="Auto")
public class MoveRobot extends LinearOpMode {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        runtime.reset();



    }
    public static double[] driveAngular(double power, double angle) { // angle will be 0 at forwards,
        // rotate left will be 90 degrees
        double velocity_x = power * Math.sin(angle);
        double velocity_y = power * Math.cos(angle);
        double[] powers = new double[4];
        powers[0] = velocity_x + velocity_y;
        powers[1] = -1 * (velocity_x - velocity_y);
        powers[2] = velocity_x + velocity_y;
        powers[3] = 1 * (velocity_x - velocity_y);
        return powers;

    }
}
