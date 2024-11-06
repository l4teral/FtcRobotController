package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TestMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "test");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();
        // Keys that do not work: 0 p P : ; - / = ? ()

        while (opModeIsActive()){
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double current = motor.getCurrent(CurrentUnit.MILLIAMPS);
            // double power = current / voltage; // power * voltage = current

            // double velocity = motor.getVelocity(); // angular velocity

            // power           =  force         *  target velocity
            // (kg * m^2)/s^3  =  (kg * m)/s^2  *  m/s

            double dutyCycle = -gamepad1.left_stick_y;
            // is it counterclockwise

            motor.setPower(dutyCycle);

            // telemetry but that apparently is not needed for this test motor
            // telemetry is basically what is printed to the console, right?
        }
    }
}
