package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * DriveSubsystem controls the robot's drive train consisting of 4 mecanum wheels:
 * frontLeft, frontRight, backLeft, and backRight.
 */
public class DriveSubsystem implements Subsystem {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    /**
     * Initializes the drive motors using the provided HardwareMap.
     *
     * @param hardwareMap the FTC HardwareMap instance
     */
    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse the left motors so forward power drives all wheels forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set brake mode so wheels stop quickly when power is zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders by default (can be updated to RUN_USING_ENCODER if needed)
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Drives the robot in a robot-centric manner using axial (forward),
     * lateral (strafe), and yaw (turn) inputs.
     *
     * @param forward positive values move the robot forward (+Y)
     * @param strafe  positive values strafe the robot right (+X)
     * @param turn    positive values turn the robot right/clockwise (+Yaw)
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        // Normalize motor powers if any exceeds 1.0
        double maxPower = Math.max(1.0, Math.max(
                Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))
        ));

        frontLeft.setPower(flPower / maxPower);
        frontRight.setPower(frPower / maxPower);
        backLeft.setPower(blPower / maxPower);
        backRight.setPower(brPower / maxPower);
    }

    /**
     * Stops all drive motors.
     */
    public void stop() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    public DcMotor getFrontLeft() {
        return frontLeft;
    }

    public DcMotor getFrontRight() {
        return frontRight;
    }

    public DcMotor getBackLeft() {
        return backLeft;
    }

    public DcMotor getBackRight() {
        return backRight;
    }
}
