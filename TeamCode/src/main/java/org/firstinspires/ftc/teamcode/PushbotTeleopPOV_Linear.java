package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(group = "Pushbot", name = "Teyanna: Teleop POV")
public class PushbotTeleopPOV_Linear extends LinearOpMode {
    final double CLAW_SPEED = 0.02d;
    boolean IntakePress = false;
    boolean IntakeRunning = false;
    boolean ShooterPress = false;
    boolean ShooterRunning = false;
    double clawOffset = LynxServoController.apiPositionFirst;
    HardwarePushbot robot = new HardwarePushbot();

    public void runOpMode() {
        char c;
        this.robot.init(this.hardwareMap, this, false);
        this.telemetry.addData("Say", (Object) "Hello Driver");
        this.telemetry.update();
        waitForStart();
        this.robot.TurningWithGyro(180.0d);
        while (opModeIsActive()) {
            double drive = (double) this.gamepad1.left_stick_y;
            double turn = ((double) (-this.gamepad1.left_stick_x)) / 4.0d;
            double left = drive + turn;
            double right = drive - turn;
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0d) {
                left /= max;
                right /= max;
            }
            double d = turn;
            if (this.gamepad1.left_bumper) {
                this.robot.leftFrontDrive.setPower(-0.5d);
                this.robot.leftBackDrive.setPower(0.5d);
                this.robot.rightFrontDrive.setPower(0.5d);
                this.robot.rightBackDrive.setPower(-0.5d);
            } else if (this.gamepad1.right_bumper) {
                this.robot.leftFrontDrive.setPower(0.5d);
                this.robot.leftBackDrive.setPower(-0.5d);
                this.robot.rightFrontDrive.setPower(-0.5d);
                this.robot.rightBackDrive.setPower(0.5d);
            } else {
                this.robot.leftFrontDrive.setPower(left / 1.0d);
                this.robot.leftBackDrive.setPower(left / 1.0d);
                this.robot.rightFrontDrive.setPower(right / 1.0d);
                this.robot.rightBackDrive.setPower(right / 1.0d);
            }
            if (this.gamepad1.dpad_right) {
                this.robot.grabberservo.setPosition(0.5d);
            } else if (this.gamepad1.dpad_left) {
                this.robot.grabberservo.setPosition(0.1d);
            }
            if (this.gamepad2.dpad_right) {
                this.robot.dropperservo.setPosition(0.6d);
            } else if (this.gamepad2.dpad_left) {
                this.robot.dropperservo.setPosition(0.2d);
            }
            if (this.gamepad1.f123x && !this.ShooterPress) {
                if (!this.ShooterRunning) {
                    this.robot.shooter.setPower(1.0d);
                    this.ShooterRunning = true;
                } else {
                    this.robot.shooter.setPower(LynxServoController.apiPositionFirst);
                    this.ShooterRunning = false;
                }
            }
            if (this.gamepad1.f120b && !this.IntakePress) {
                if (!this.IntakeRunning) {
                    this.robot.intake.setPower(1.0d);
                    this.IntakeRunning = true;
                } else {
                    this.robot.intake.setPower(LynxServoController.apiPositionFirst);
                    this.IntakeRunning = false;
                }
            }
            this.ShooterPress = this.gamepad1.f123x;
            this.IntakePress = this.gamepad1.f120b;
            if (this.gamepad1.f119a) {
                this.robot.intake.setPower(-1.0d);
            } else if (!this.gamepad1.f119a && this.robot.intake.getPower() < LynxServoController.apiPositionFirst) {
                this.robot.intake.setPower(LynxServoController.apiPositionFirst);
            }
            if (this.gamepad1.dpad_up) {
                this.robot.liftPosition(true);
                c = 0;
            } else if (this.gamepad1.dpad_down) {
                c = 0;
                this.robot.liftPosition(false);
            } else {
                c = 0;
            }
            Telemetry telemetry = this.telemetry;
            Object[] objArr = new Object[1];
            objArr[c] = Double.valueOf(left);
            telemetry.addData("left", "%.2f", objArr);
            this.telemetry.addData("distance", "%.2f", Double.valueOf(this.robot.sensorRange.getDistance(DistanceUnit.CM)));
            this.telemetry.addData("right", "%.2f", Double.valueOf(right));
            this.telemetry.addData("angle", "%.2f", Float.valueOf(this.robot.FixAngle()));
            this.telemetry.addData("lift", "%d LiftHite", Integer.valueOf(this.robot.lift.getCurrentPosition()));
            this.telemetry.update();
            sleep(50);
        }
    }
}
