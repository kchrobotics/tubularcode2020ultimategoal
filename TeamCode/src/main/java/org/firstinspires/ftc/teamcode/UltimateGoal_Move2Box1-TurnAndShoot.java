package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(group = "Pushbot", name = "Move2Boxes ")
public class UltimateGoal_Move2Box1 extends LinearOpMode {
    static final double APPROACH_SPEED = 0.5d;
    static final double WHITE_THRESHOLD = 0.2d;
    HardwarePushbot robot = new HardwarePushbot();

    public void runOpMode() {
        this.robot.init(this.hardwareMap, this, true);
        this.robot.dropperservo.setPosition(WHITE_THRESHOLD);
        this.telemetry.addData("Status", (Object) "Ready to run");
        this.telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            idle();
        }
        int rings = this.robot.highrings();
        Telemetry telemetry = this.telemetry;
        telemetry.addData("Status", (Object) "I c this many rings" + rings);
        this.telemetry.update();
        if (rings == 4) {
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, -0.6d, 4.5d);
            this.robot.FollowingAngleDistance(LynxServoController.apiPositionFirst, -0.6d, 25.0d);
            this.robot.TurningWithGyro(-90.0d);
            this.robot.FollowingAngleDistanceAway(-90.0d, 0.6d, 45.0d);
            this.robot.dropperservo.setPosition(0.6d);
            this.robot.TurningWithGyro(LynxServoController.apiPositionFirst);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, 0.6d, 3.0d);
            sleep(10000);
            this.robot.TurningWithGyro(179.0d);
            sleep(500);
            this.robot.shooter.setPower(1.0d);
            sleep(2000);
            this.robot.intake.setPower(-1.0d);
            sleep(4000);
            this.robot.shooter.setPower(LynxServoController.apiPositionFirst);
            this.robot.intake.setPower(LynxServoController.apiPositionFirst);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, -0.6d, 3.0d);
        } else if (rings == 0) {
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, -0.6d, 4.5d);
            this.robot.TurningWithGyro(-90.0d);
            this.robot.dropperservo.setPosition(0.6d);
            this.robot.FollowingAngleDistanceAway(-90.0d, 0.6d, 100.0d);
            this.robot.TurningWithGyro(LynxServoController.apiPositionFirst);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, 0.6d, 1.5d);
            this.robot.TurningWithGyro(179.0d);
            sleep(500);
            this.robot.shooter.setPower(1.0d);
            sleep(2000);
            this.robot.intake.setPower(-1.0d);
            sleep(4000);
            this.robot.shooter.setPower(LynxServoController.apiPositionFirst);
            this.robot.intake.setPower(LynxServoController.apiPositionFirst);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, -0.6d, 3.0d);
        } else {
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, -0.6d, 4.5d);
            this.robot.FollowingAngleDistance(LynxServoController.apiPositionFirst, -0.6d, 90.0d);
            this.robot.TurningWithGyro(-90.0d);
            this.robot.FollowingAngleDistanceAway(-90.0d, 0.6d, 125.0d);
            this.robot.TurningWithGyro(LynxServoController.apiPositionFirst);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, 0.6d, 1.0d);
            this.robot.dropperservo.setPosition(0.6d);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, 0.6d, 1.5d);
            this.robot.TurningWithGyro(170.0d);
            sleep(500);
            this.robot.shooter.setPower(1.0d);
            sleep(2000);
            this.robot.intake.setPower(-1.0d);
            sleep(4000);
            this.robot.shooter.setPower(LynxServoController.apiPositionFirst);
            this.robot.intake.setPower(LynxServoController.apiPositionFirst);
            this.robot.FollowingAngle(LynxServoController.apiPositionFirst, -0.6d, 3.0d);
        }
    }
}
