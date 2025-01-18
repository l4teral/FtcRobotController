package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class RobotAutonomous extends LinearOpMode {
    RobotChassisDrive robot = new RobotChassisDrive();
    RobotArm arm = new RobotArm();
    RobotIntake intake = new RobotIntake();

    // variables
    double headingError = 0.0;
    double targetHeading = 0.0;
    double driveSpeed = 0.0;
    double turnSpeed = 0.0;
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;
    double leftTarget = 0.0;
    double rightTarget = 0.0;

    // constants
    // instead of all caps with underscores I am beginning these names with capital letters
    // underscore is not working

    final double CountsPerChassisMotorRev = 0.0;
    final double DriveGearReduction = 1.0;
    final double WheelDiameterInches = 4.0;
    final double CountsPerInch = (CountsPerChassisMotorRev*DriveGearReduction)/(WheelDiameterInches*3.14159);

    final double DriveSpeed = 0.4;
    final double TurnSpeed = 0.2;
    final double HeadingThreshold = 1.0;

    final double PTurnGain = 0.2;
    final double PDriveGain = 0.3;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    IMU imu = hardwareMap.get(IMU.class, "imu");

    int[] odometryReadings = new int[10];
    int curReading = 0;

    DcMotorEx frontOdometry = null;
    DcMotorEx backOdometry = null;

    // basic functions

    // Keys that do not work: 0 p P : ; - / = ? () ’ " [] + {} _

    public void driveStraight(double maxDriveSpeed, double distance, double heading){
        // determine the new target position, and pass to motor controller
        double moveCounts = distance * CountsPerInch;
        leftTarget = robot.FLMotor.getCurrentPosition() * moveCounts;
        rightTarget = robot.FRMotor.getCurrentPosition() * moveCounts;

        robot.FLMotor.setTargetPosition((int) leftTarget);
        robot.FRMotor.setTargetPosition((int) rightTarget);

        for (int i = 0; i < robot.ChassisMotors.size(); i++) {
            robot.ChassisMotors.get(i).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        maxDriveSpeed = Math.abs(maxDriveSpeed);

        moveRobot(maxDriveSpeed, 0);

        while (robot.FLMotor.isBusy() && robot.FRMotor.isBusy() && robot.BLMotor.isBusy() && robot.BRMotor.isBusy()) {
            turnSpeed = getSteeringCorrection(heading, PTurnGain);

            if (distance < 0) {
                turnSpeed = turnSpeed * -1;
            }

            moveRobot(driveSpeed, turnSpeed);
            sendTelemetry(true);
        }

        moveRobot(0, 0);

        for (int i = 0; i < robot.ChassisMotors.size(); i++) {
            robot.ChassisMotors.get(i).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // Keys that do not work: 0 p P : ; - / = ? () ’ " [] + {} _
    public void turnToHeading(double maxturnSpeed, double heading){
        turnSpeed = getSteeringCorrection(heading, PDriveGain);

        while (Math.abs(headingError) > HeadingThreshold) {
            turnSpeed = getSteeringCorrection(heading, PTurnGain);
            if (turnSpeed < -1) {
                turnSpeed = -1;
            } else if (turnSpeed > 1) {
                turnSpeed = 1;
            }

            moveRobot(0, turnSpeed);
            sendTelemetry(false);

            moveRobot(0, 0);
        }
    }

    public void holdHeading(double maxturnSpeed, double heading, double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.seconds() < holdTime) {
            turnSpeed = getSteeringCorrection(heading, PTurnGain);
            if (turnSpeed < -1) {
                turnSpeed = -1;
            } else if (turnSpeed > 1) {
                turnSpeed = 1;
            }
            moveRobot(0, turnSpeed);
            sendTelemetry(false);
        }
    }

    // Keys that do not work: 0 p P : ; - / = ? () ’ " [] + {} _

    public void moveRobot(double drive, double turn){
        driveSpeed = drive;
        turnSpeed = turn;

        leftSpeed = drive + turn;
        rightSpeed = drive - turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

        if (max > 1) {
            leftSpeed = leftSpeed / max;
            rightSpeed = rightSpeed / max;
        }

        for (int i = 0; i < 2; i++) {
            robot.ChassisLeftMotors.get(i).setPower(drive + turn);
            robot.ChassisRightMotors.get(i).setPower(drive - turn);
        }
    }

    // Keys that do not work: 0 p P : ; - / = ? () ’ " [] + {} _

    public double getSteeringCorrection(double desiredHeading, double proportionalGain){
        targetHeading = desiredHeading;

        headingError = targetHeading - getHeading();

        while (headingError > 180) {
            headingError = headingError - 360;
        }
        while (headingError <= 180) {
            headingError = headingError + 360;
        }

        double steeringCorrection = headingError * proportionalGain;

        if (steeringCorrection < -1) {
            steeringCorrection = -1;
        } else if (steeringCorrection > 1) {
            steeringCorrection = 1;
        }

        return steeringCorrection;
    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    // Keys that do not work: 0 p P : ; - / = ? () ’ "" [] + {} _

    public void sendTelemetry(boolean straight){
        if (straight) {
            telemetry.addData("Motion","Drive Straight");
            telemetry.addData("Target Position L:R", "%.0f:%.0f", leftTarget, rightTarget);
            telemetry.addData("Actual Position L:R", "%.0f:%.0f", robot.FLMotor.getCurrentPosition(), robot.FRMotor.getCurrentPosition());
        } else {
            telemetry.addData("Motion","Turning");
        }

        telemetry.addData("Heading - Target : Current", "%.2f:%.2f", targetHeading, getHeading());
        telemetry.addData("Error : Steer Power", "%.1f:%.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%.2f:%.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }


        public void runOpMode(){
        // Keys that do not work: 0 p P : ; - / = ? () " ’ [] +
            robot.init(hardwareMap,  telemetry);
            arm.init(hardwareMap,  telemetry);
            intake.init(hardwareMap,  telemetry);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();

            // heading is the direction between 0 and 359.99 degrees
            // higher heading means more clockwise

            telemetry.addData("> Robot Heading", "%.0f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            telemetry.addLine("> Press START to operate the robot.");
            telemetry.update();

            waitForStart();

            odometryReadings[curReading] = frontOdometry.getCurrentPosition();

            int oldestReading = (curReading+1) % odometryReadings.length;
            int dif = odometryReadings[oldestReading] - odometryReadings[curReading];

            telemetry.addData("Odometry Difference", String.valueOf(dif));
            telemetry.addData("Arm Position", String.valueOf(arm.wormGearMotor.getCurrentPosition()));
            telemetry.addData("Arm Extender Position", String.valueOf(arm.extender.getCurrentPosition()));
            telemetry.update();

            // Keys that do not work: 0 p P : ; - / = ? () ’ "" [] + {} _

            // HERE WE BEGIN THE MOTION

            // strategy: put four samples in the low basket (16), hang (15), and park (3)
            // 34 points

            //  1. Start at F2 and face left
            turnToHeading(TurnSpeed, -25);
            holdHeading(DriveSpeed, -25, .5);

            //  2. collect sample with arm and servo
            //     (determine best arm and servo orientation for picking up samples)

            //  3. head toward third spike from wall, then go back

            //  4. head toward second spike from wall, then go back

            //  5. head toward spike closest to wall,

            //  6. back up and go to ascent zone, 1

            //  7. face first spike, second sample

            //  8. hold heading for a half second

            //  9. drive toward second spike

            // 10. drive toward net zone

            // 11. back up and go to ascent zone, 2

            // 12. face front

            // 13. face second spike and net zone

            // 14. push third spike into net zone

            // 15.
    }
}