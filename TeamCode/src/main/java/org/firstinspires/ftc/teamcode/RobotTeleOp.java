package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class RobotTeleOp extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init(){
        robot.init(hardwareMap, telemetry);

        // Send a telemetry message to signal the end of initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void updateMotorPower(){
        telemetry.addData("FL Motor Power", robot.FLMotor.getPower());
        telemetry.addData("FR Motor Power", robot.FRMotor.getPower());
        telemetry.addData("BL Motor Power", robot.BLMotor.getPower());
        telemetry.addData("BR Motor Power", robot.BRMotor.getPower());
        telemetry.update();
    }

    @Override
    public void loop(){

        // Keys that do not work: 0 p P : ; - / = ? () " ’ []

        Gamepad main = gamepad1;

        if (gamepad1.a && gamepad1.y) {
            main = gamepad1;
        } else if (gamepad2.a && gamepad2.y) {
            main = gamepad2;
        }

        // drive is controlled by left stick of game
        robot.drive(main.left_stick_y);
        updateMotorPower();

        // strafe is controlled by left stick
        robot.strafe(main.left_stick_x);
        updateMotorPower();

        // turn is controlled by right stick
        robot.turn(main.right_stick_x);
        updateMotorPower();

        // extend, retract, nonea

        /*
        if (main.right_stick_x < 0){
            robot.turn(main.right_stick_x, true); // turns left
        } else if (main.right_stick_x > 0){
            robot.turn(main.right_stick_x, false); // turns right
        } else {
            arm.sto
        }
        */

        /*
        if (gamepad2.right_bumper) {
            robot.servo1.setPosition(1);
        } else if (gamepad2.left_bumper) {
            robot.servo1.setPosition(0);
            robot.servo2.setPosition(0);
        }
        */

        // Keys that do not work: 0 p P : ; - / = ? () " ’ []
    }
}
