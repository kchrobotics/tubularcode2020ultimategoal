package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(group = "Pushbot", name = "Teyanna: Turn Right 90")
public class PushbotAutoDriveToLine_Linear extends LinearOpMode {
    static final double APPROACH_SPEED = 0.5d;
    static final double WHITE_THRESHOLD = 0.2d;
    HardwarePushbot robot = new HardwarePushbot();

    public void runOpMode() {
        this.robot.init(this.hardwareMap, this, false);
        this.telemetry.addData("Status", (Object) "Ready to run");
        this.telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            idle();
        }
        int rings = this.robot.highrings();
        Telemetry telemetry = this.telemetry;
        telemetry.addData("rings", (Object) "rings" + rings);
        this.telemetry.update();
    }
}
