package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotIntake {
    public CRServo wheel;

    public void init(HardwareMap HM, Telemetry telemetry){
            try {
                wheel = HM.get(CRServo.class, "wheel");

                // initialized?
                if (wheel == null){
                    telemetry.addData("Servo Error", "wheel not initialized");
                    throw new RuntimeException("Servo initialization failed");
                }
            } catch (Exception exception) {
                telemetry.addData("Error", "Initialization failed: " + exception.getMessage());
                telemetry.update();
            }
    }

    // Keys that do not work: 0 p P : ; - / = ? () " ’ [] + {}
}
