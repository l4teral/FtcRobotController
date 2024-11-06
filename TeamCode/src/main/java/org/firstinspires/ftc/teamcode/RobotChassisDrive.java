package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

// Keys that do not work: 0 p P : ; - / = ? ()

public class RobotChassisDrive {
    DcMotorEx FLMotor; // front left
    DcMotorEx FRMotor; // front right
    DcMotorEx BLMotor; // back left
    DcMotorEx BRMotor; // back right

    ArrayList<DcMotorEx> ChassisMotors;

    public void init(HardwareMap HM, Telemetry telemetry){
        try {
            FLMotor = HM.get(DcMotorEx.class, "fl"); // front left
            FRMotor = HM.get(DcMotorEx.class, "fr"); // front right
            BLMotor = HM.get(DcMotorEx.class, "bl"); // back left
            BRMotor = HM.get(DcMotorEx.class, "br"); // back right

            // array list of chassis motors
            ChassisMotors = new ArrayList<>();
            ChassisMotors.add(FLMotor);
            ChassisMotors.add(FRMotor);
            ChassisMotors.add(BLMotor);
            ChassisMotors.add(BRMotor);

            // initialized?
            for (int i = 0; i < ChassisMotors.size(); i++){
                if (ChassisMotors.get(i) == null){
                    telemetry.addData("Motor Error", ChassisMotors.get(i).getDeviceName() + " not initialized");
                    throw new RuntimeException("Motor initialization failed");
                }
            }

            // left motors forward (CCW is front)
            FLMotor.setDirection(DcMotorEx.Direction.FORWARD);
            BLMotor.setDirection(DcMotorEx.Direction.FORWARD);

            // right motors backward (CW is front)
            FRMotor.setDirection(DcMotorEx.Direction.REVERSE);
            BRMotor.setDirection(DcMotorEx.Direction.REVERSE);

            for (int i = 0; i < ChassisMotors.size(); i++) {
                ChassisMotors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } catch (Exception exception) {
            telemetry.addData("Error", "Initialization failed: " + exception.getMessage());
            telemetry.update();
        }
    }

    // set power
    public void chassisMotorsSetPower(double p){
        for (int i = 0; i < ChassisMotors.size(); i++) {
            ChassisMotors.get(i).setPower(p);
        }
    }

    // fwd?
    public void drive(double power){
        chassisMotorsSetPower(power);
    }

    // left?
    public void turn(double power){
        for (int i = 0; i < ChassisMotors.size(); i+=2) {
            ChassisMotors.get(i).setPower(-power);
        }
        for (int i = 1; i < ChassisMotors.size(); i+=2) {
            ChassisMotors.get(i).setPower(power);
        }
    }

    // strafe function: positive power for right, negative for left
    public void strafe(double power) {
        FLMotor.setPower(power);
        FRMotor.setPower(-power);
        BLMotor.setPower(-power);
        BRMotor.setPower(power);
    }
}