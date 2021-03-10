/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teyanna: Teleop POV", group="Pushbot")

public class PushbotTeleopPOV_Linear extends LinearOpMode {
boolean ShooterPress = false;
    boolean ShooterRunning = false;
    boolean IntakePress = false;
    boolean IntakeRunning = false;

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;
            turn = turn/4;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            //Drift controls
            //Drift Left
            if (gamepad1.left_bumper){
                robot.leftFrontDrive.setPower(-0.5);
                robot.leftBackDrive.setPower(0.5);
                robot.rightFrontDrive.setPower(0.5);
                robot.rightBackDrive.setPower(-0.5);
            }
            //Drift Right
            else if (gamepad1.right_bumper){
                robot.leftFrontDrive.setPower(0.5);
                robot.leftBackDrive.setPower(-0.5);
                robot.rightFrontDrive.setPower(-0.5);
                robot.rightBackDrive.setPower(0.5);

            }
            else {
                // Output the safe values to the motor drives.
                robot.leftFrontDrive.setPower(left/1);
                robot.leftBackDrive.setPower(left/1);
                robot.rightFrontDrive.setPower(right/1);
                robot.rightBackDrive.setPower(right/1);

            }

            if(gamepad1.dpad_right){
            robot.grabberservo.setPosition(0.6);
            }
            else if (gamepad1.dpad_left){
                robot.grabberservo.setPosition(0);
            }



            if (gamepad1.x){
                if (ShooterPress == false){
                    //Time to POWERDOWN!
                    if(ShooterRunning == false){
                        robot.shooter.setPower(1);
                        ShooterRunning = true;
                    }
                    else{
                        robot.shooter.setPower(0);
                        ShooterRunning = false;
                    }
                }
            }

            if (gamepad1.b){
                if (IntakePress == false){
                    //Time to POWERDOWN!
                    if(IntakeRunning == false){
                        robot.intake.setPower(1);
                        IntakeRunning = true;
                    }
                    else{
                        robot.intake.setPower(0);
                        IntakeRunning = false;
                    }
                }
            }

                ShooterPress = gamepad1.x;
                IntakePress = gamepad1.b;
            if (gamepad1.a){
                robot.intake.setPower(-1);
            }
            else if (gamepad1.a == false && robot.intake.getPower() < 0){
                robot.intake.setPower(0);
            }

            if (gamepad1.dpad_up){
                //robot.lift.setPower(1);
                robot.liftPosition(true);
            }
            else if (gamepad1.dpad_down){
                //robot.lift.setPower(-1);
                robot.liftPosition(false);
            }
            else{
                //robot.lift.setPower(0);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("distance",  "%.2f", (robot.rangeSensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("angle",  "%.2f", robot.FixAngle());
            telemetry.addData("lift",  "%dTeyTey", robot.lift.getCurrentPosition());
            telemetry.update();


            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
