package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RobotAutonomousLeft extends LinearOpMode {

    // Keys that do not work: 0 p P : ; - / = ? () ’ "" [] + {} _

    double y = 0.0;
    double x = 0.0;
    double cw = 0.0;

    RobotChassisDrive robot = new RobotChassisDrive();

    public void runOpMode() throws InterruptedException {
        // Initialize the drive system
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0)); // Starting position (x, y, heading)

        // Wait for the start signal
        waitForStart();

        while (opModeIsActive()){

            sleep(2000);

            x = 0.5;

            robot.FLMotor.setPower(y - x - cw);
            robot.FRMotor.setPower(-y - x - cw);
            robot.BLMotor.setPower(y + x - cw);
            robot.BRMotor.setPower(-y + x - cw);

            sleep(1500);

            stop();
        }
    }
}