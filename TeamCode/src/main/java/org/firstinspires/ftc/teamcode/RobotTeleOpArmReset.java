package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class RobotTeleOpArmReset extends OpMode {
    RobotChassisDrive robot = new RobotChassisDrive();
    RobotArm arm = new RobotArm();
    RobotIntake intake = new RobotIntake();

    @Override
    public void init(){
        robot.init(hardwareMap,  telemetry);
        arm.init(hardwareMap,  telemetry);
        intake.init(hardwareMap,  telemetry);
        double voltage = hardwareMap.voltageSensor.get("Expansion Hub 2").getVoltage();

        telemetry.addData("Voltage", voltage);
        telemetry.update();

        // Send a telemetry message to signal the end of initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void updateChassisMotorPower(){
        telemetry.addData("FL Motor Power", robot.FLMotor.getPower());
        telemetry.addData("FR Motor Power", robot.FRMotor.getPower());
        telemetry.addData("BL Motor Power", robot.BLMotor.getPower());
        telemetry.addData("BR Motor Power", robot.BRMotor.getPower());
        telemetry.update();
    }

    @Override
    public void loop(){

        // Keys that do not work: 0 p P : ; - / = ? () "" ’

        Gamepad chassisPad = gamepad1;
        Gamepad armPad = gamepad2;
        Gamepad temp;

        if (armPad.y && armPad.x) {
            temp = chassisPad;
            chassisPad = armPad;
            armPad = temp;
        }

        // gamepad1 for controls
        double y = chassisPad.left_stick_y;
        double x = chassisPad.left_stick_x;
        double cw = chassisPad.right_stick_x;

        robot.FLMotor.setPower(y - x - cw);
        robot.FRMotor.setPower(-y - x - cw);
        robot.BLMotor.setPower(y + x - cw);
        robot.BRMotor.setPower(-y + x - cw);
        updateChassisMotorPower();

        // extend, retract, or neither

        int armPosition = arm.wormGearMotor.getCurrentPosition();

        arm.wormGearMotor.setPower(-armPad.left_stick_y);

        telemetry.addData("Arm Position", armPosition);
        // extend, retract, or neither

        if (armPad.right_bumper) {
            arm.extend();
        } else if (armPad.left_bumper) {
            arm.retract();
        } else {
            arm.stop();
        }

        telemetry.addData("Arm Extension Position", arm.extender.getCurrentPosition());
        telemetry.addData("Arm Extension Power", arm.extender.getPower());

        intake.wheel.setPower(armPad.a ? -1 : armPad.b ? 1 : 0);
        telemetry.addData("Wheel Power", String.valueOf(intake.wheel.getPower()));

        // Keys that do not work: 0 p P : ; - / = ? () " ’ [] {}
    }
}