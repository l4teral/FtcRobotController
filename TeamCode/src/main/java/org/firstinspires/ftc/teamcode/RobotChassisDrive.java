package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;

public class RobotChassisDrive {
    DcMotorEx FLMotor; // front left
    DcMotorEx FRMotor; // front right
    DcMotorEx BLMotor; // back left
    DcMotorEx BRMotor; // back right

    ArrayList<DcMotorEx> ChassisMotors;
    ArrayList<DcMotorEx> ChassisLeftMotors;
    ArrayList<DcMotorEx> ChassisRightMotors;

    public void init(HardwareMap HM, Telemetry telemetry){
        try {
            FLMotor = HM.get(DcMotorEx.class, "fl"); // front left
            FRMotor = HM.get(DcMotorEx.class, "fr"); // front right
            BLMotor = HM.get(DcMotorEx.class, "bl"); // back left
            BRMotor = HM.get(DcMotorEx.class, "br"); // back right

            // all motors
            ChassisMotors = new ArrayList<>();
            ChassisMotors.add(FLMotor);
            ChassisMotors.add(FRMotor);
            ChassisMotors.add(BRMotor);
            ChassisMotors.add(BLMotor);

            // left motors
            ChassisLeftMotors.add(FLMotor);
            ChassisLeftMotors.add(BLMotor);

            // right motors
            ChassisRightMotors.add(FRMotor);
            ChassisRightMotors.add(BRMotor);

            // initialized?
            for (int i = 0; i < ChassisMotors.size(); i++){
                if (ChassisMotors.get(i) == null){
                    telemetry.addData("Motor Error", ChassisMotors.get(i).getDeviceName() + " not initialized");
                    throw new RuntimeException("Motor initialization failed");
                }
            }

            // left motors forward (CCW is front)
            FLMotor.setDirection(DcMotor.Direction.FORWARD);
            FRMotor.setDirection(DcMotor.Direction.FORWARD);
            BLMotor.setDirection(DcMotor.Direction.REVERSE);
            BRMotor.setDirection(DcMotor.Direction.REVERSE);

            for (int i = 0; i < ChassisMotors.size(); i++) {
                ChassisMotors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        } catch (Exception exception) {
            telemetry.addData("Error", "Initialization failed: " + exception.getMessage());
            telemetry.update();
        }
    }
}