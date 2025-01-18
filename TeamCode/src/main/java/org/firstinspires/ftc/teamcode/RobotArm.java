package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;


public class RobotArm {
    DcMotorEx wormGearMotor;
    DcMotorEx extender;

    double unit;

    ArrayList<DcMotorEx> armMotors;

    public void init(HardwareMap HM, Telemetry telemetry){
        try {
            wormGearMotor = HM.get(DcMotorEx.class, "wgm");
            extender = HM.get(DcMotorEx.class, "ext");

            armMotors = new ArrayList<>();
            armMotors.add(wormGearMotor);
            armMotors.add(extender);

            // initialized?
            for (int i = 0; i < armMotors.size(); i++){
                if (armMotors.get(i) == null){
                    telemetry.addData("Motor Error", "wheel not initialized");
                    throw new RuntimeException("Motor initialization failed");
                }
            }

            wormGearMotor.setDirection(DcMotorEx.Direction.FORWARD);
            extender.setDirection(DcMotorEx.Direction.FORWARD);

            for (int i = 0; i < armMotors.size(); i++) {
                armMotors.get(i).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            for (int i = 0; i < armMotors.size(); i++) {
                armMotors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotors.get(i).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } catch (Exception exception) {
            telemetry.addData("Error", "Initialization failed: " + exception.getMessage());
            telemetry.update();
        }
    }

    // Keys that do not work: 0 p P : ; - / = ? () " ’ [] + {}

    // extender

    public void extend() {
        extender.setPower(1.0); // Full speed cw
    }

    public void retract() {
        extender.setPower(-1.0); // Full speed ccw
    }

    public void stop() {
        extender.setPower(0.0); // Neutral (no movement)
    }

}