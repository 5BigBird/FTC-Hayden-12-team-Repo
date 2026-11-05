package org.firstinspires.ftc.teamcode.automode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Luc auto v1 test", group="Auto")
public class Cannon {

    // Hardware Declarations
    private DcMotorEx flywheelMotor;
    private Servo feederServo; // Feeder/kicker pushing pollen into the flywheel

    // Target Velocity (Ticks per second) - Adjust based on your shooting distance
    private double TARGET_VELOCITY;
    // Tolerance window (Allow shooting if flywheel is within 50 ticks of target)
    private double VELOCITY_TOLERANCE = 50.0;

    // Servo positions for feeding pollen
    private final double FEEDER_REST_POS = 0.0;
    private final double FEEDER_PUSH_POS = 1.0;

    private final double x_offset;
    private final double y_offset;
    private final double z_offset;

    public Cannon(double x, double y, double z) {
        this.x_offset = x;
        this.y_offset = y;
        this.z_offset = z;
    }
    public double get_shoot_pos_adjustment(double[] distToAprilTag, double angle) {
        double shooterAngle = 55; // angle in degrees from the horizontal
        double initVelo = 2; // (m/s)
        double VeloX = Math.cos(shooterAngle) * initVelo;
        double initVeloY = Math.sin(shooterAngle) * initVelo;
        double d_y = distToAprilTag[1];
        double accel = -9.8;
        // WHAT I KNOW:
            // accel
            // delta x and y
            // start velocity x
        double t = (-1 * initVeloY + Math.sqrt((initVeloY * initVeloY) - (4 * (accel / 2) * d_y))) / accel; // this is how long the ball needs to be in the air to make it in

        double d_x = (VeloX * t); // this is where I want to be




        // Calculate centrifugal forces on the ball given by the motor spinning the cannon to achieve
        // a high enough initial velocity that doesn't go too high or low (perhaps in the middle)

        // account for robot velocity? maybe a future thing, who knows
    }
}