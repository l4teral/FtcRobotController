package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotArm {

    // Keys that do not work: 0 p P : ; - / = ? () " ’ [] + {}

    /*     objectives of the arm:
     *      1.   transferring samples to board
     *      2.   specimen
     *      2.   hanging (1st or 2nd rung)
     */

    /*     functions of the arm:
     *      1.   extending (via CW rotation) and retracting (via CCW rotation)
     *      2.
     */

    // one servo and one motor
    DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "");
    Servo armServo = hardwareMap.get(Servo.class, "");

    /* final double ticksPerDeg =
            28 // number of encoder ticks per rotation of the bare motor
                    * (250047.0 / 4913.0) // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * (100.0 / 20.0) // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * (1/360.0); // we want ticks per degree, not per rotation
     */

    // constants for moving power
    private static final double extend = 1.0;
    private static final double retract = -1.0;
    private static final double stop = 0.0;

    public void init(){
        armServo.setPosition(0.5);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void rotateCCW() {
        armServo.setPosition(0.0); // Full speed counterclockwise
    }

    public void rotateCW() {
        armServo.setPosition(1.0); // Full speed clockwise
    }

    public void stopArm() {
        armServo.setPosition(0.5); // Neutral (no movement)
    }

}