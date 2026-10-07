package org.firstinspires.ftc.teamcode.automode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Luc auto v1 test", group="Auto")
public class auto extends LinearOpMode {
    private final double PI = 3.1415926535;
    private DcMotor leftFrontDrive  = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightBackDrive  = null;

    // Declare Subsystem Motors (e.g., Flywheel/Shooter for Pollen)
    private DcMotor pollenShooter   = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors so forward power moves the bot forward
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized. Waiting for Start...");
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();
        runtime.reset();

        driveForward(0.5);
        sleep(500);
        stopDriving();

        driveBackward(0.5);
        sleep(500);
        stopDriving();

        for (int i = 0; i < 360; i += 5) {
            stopDriving();
            driveAngular(0.2, i * PI / 180);
            sleep(100);
        }
    }

    public void driveForward(double power) { // go forwards
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void driveBackward(double power) { // go backwards
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }
    public void driveAngular(double power, double angle) { // angle will be 0 at forwards,
                                                           // rotate left will be 90 degrees
        double velocity_x = power * Math.sin(angle);
        double velocity_y = power * Math.cos(angle);
        leftFrontDrive.setPower(velocity_x + velocity_y);
        rightFrontDrive.setPower(-1 * (velocity_x - velocity_y));
        leftBackDrive.setPower(velocity_x + velocity_y);
        rightBackDrive.setPower(-1 * (velocity_x - velocity_y));
    }
    public void stopDriving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
