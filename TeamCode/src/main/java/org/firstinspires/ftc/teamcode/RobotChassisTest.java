package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotChassisTest extends OpMode {
    RobotChassisDrive chassis = new RobotChassisDrive();

    @Override
    public void init(){
        chassis.init(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        if (gamepad1.left_stick_y > 0){
            chassis.drive(gamepad1.left_stick_y); // forward
        } else if (gamepad1.left_stick_y < 0){
            chassis.drive(gamepad1.left_stick_y); // backward
        }

        if (gamepad1.left_stick_y > 0){
            chassis.turn(gamepad1.left_stick_x); // turns left
        } else if (gamepad1.left_stick_y < 0){
            chassis.turn(gamepad1.left_stick_x); // turns right
        }

    }
}
