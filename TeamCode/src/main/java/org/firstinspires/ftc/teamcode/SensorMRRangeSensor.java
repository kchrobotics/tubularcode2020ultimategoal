package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "Sensor", name = "Teyanna'sSensor: MR range sensor")
public class SensorMRRangeSensor extends LinearOpMode {
    ModernRoboticsI2cRangeSensor rangeSensor;

    public void runOpMode() {
        this.rangeSensor = (ModernRoboticsI2cRangeSensor) this.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        waitForStart();
        while (opModeIsActive()) {
            this.telemetry.addData("Teyanna ultrasonic %.2f cm", (Object) Integer.valueOf(this.rangeSensor.rawUltrasonic()));
            this.telemetry.addData("raw optical", (Object) Integer.valueOf(this.rangeSensor.rawOptical()));
            this.telemetry.addData("cm optical", "%.2f cm", Double.valueOf(this.rangeSensor.cmOptical()));
            this.telemetry.addData("cm", "%.2f cm", Double.valueOf(this.rangeSensor.getDistance(DistanceUnit.CM)));
            this.telemetry.update();
        }
    }
}
