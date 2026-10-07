package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Simple TeleOp", group = "TeleOp")
public class RobotTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {

        //init
        DriveSubsystem drive = new DriveSubsystem();
        drive.init(hardwareMap);
        DriveCommand driveCommand = new DriveCommand(
                drive,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x
        );

        waitForStart();
        
        //play
        while (opModeIsActive()) {
            driveCommand.execute();
        }

        driveCommand.end();
    }
}
