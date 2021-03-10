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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file illustrates the concept of driving up to a line and then stopping.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code shows using two different light sensors:
 *   The Primary sensor shown in this code is a legacy NXT Light sensor (called "sensor_light")
 *   Alternative "commented out" code uses a MR Optical Distance Sensor (called "sensor_ods")
 *   instead of the LEGO sensor.  Chose to use one sensor or the other.
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set half way between the light and dark values.
 *   These values can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the senso on asnd off the white line and not the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD half way between the min and max.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Move2Boxes ", group="Pushbot")


public class UltimateGoal_Move2Box1 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor

    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;

    @Override
    public void runOpMode() {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, this);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;


        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.

        



        telemetry.addData("Status", "Ready to run" );    //
        telemetry.update();

        while (!(isStarted() || isStopRequested())) {

            // Display the light level while we are waiting to start
            idle();
        }

        int rings =  4 ;   //robot.highrings();

        telemetry.addData("Status", "I c this many rings"  + rings );    //
        telemetry.update();
        sleep(2000);

            // Line 93 makes the robot move box 1

        if (rings == 4){
            //Box 1
            robot.FollowingAngleDistance(0, 0.5, 40);
            // robot.FollowingAngle(0, 0.5, 6.5);
            sleep(1000);
            robot.FollowingAngleDistanceAway(0,-0.5,120);
            robot.TurningWithGyro(-90);
            robot.FollowingAngleDistanceAway(-90,-0.5,85);

        }
        else if (rings == 0){
            //Box 3
          // robot.FollowingAngle(0,0.5,2.8);
            robot.FollowingAngleDistance(0,0.5,142);
           // sleep(2000);
                //robot.FollowingAngleDistance(0,-0.5,18);
            //Not sure if I will use this
            //robot.FollowingAngle(0,-0.5,1.0);
                robot.TurningWithGyro(-90);
            robot.FollowingAngleDistanceAway(-90,-0.5,85);
           // robot.FollowingAngle(-90,-0.5,0.5);
        }
        else {
            //Box 2
            robot.FollowingAngleDistance(0, 0.5, 85);
          //  robot.FollowingAngle(0, 0.5, 3.3);
            robot.TurningWithGyro(-90);
            robot.FollowingAngleDistanceAway(-90,-0.5,120);
            robot.TurningWithGyro(0);
            robot.FollowingAngleDistanceAway(0,-0.5,80);
            //robot.FollowingAngleDistanceAway(0,-0.5,138);
            //robot.FollowingAngleDistanceAway(-90,-0.5,85);
         //   robot.FollowingAngleDistance(90, 0.5, 50);
        }

     //

        //Line 95 makes the robot move to box 3
//



//        robot.FollowingAngle(0,-0.5,1);
//        robot.TurningWithGyro(90);
//        robot.leftFrontDrive.setPower(0);
//         robot.leftBackDrive.setPower(0);
//           robot.rightFrontDrive.setPower(0);
//           robot.rightBackDrive.setPower(0);
//            robot.FollowingAngle(90,0.5,1.2);
//        robot.TurningWithGyro(90);
//        robot.leftFrontDrive.setPower(0);
//        robot.leftBackDrive.setPower(0);
//        robot.rightFrontDrive.setPower(0);
//        robot.rightBackDrive.setPower(0);
//        robot.FollowingAngle(180,0.5,1.6);
//        robot.TurningWithGyro(0);
//        robot.FollowingAngle(0,0.5,2.8);
//        robot.TurningWithGyro(-90);
//        robot.FollowingAngle(-90,0.5,0.5);
//        robot.FollowingAngle(-90,-0.5,1.5);

        //Line 99 makes the robot move to box 2
//

    //

//
//        robot.FollowingAngle(90,0.5,1.5);
//        robot.TurningWithGyro(0);
//        robot.FollowingAngle(0,0.5, 2.3);
//        robot.FollowingAngle(0,-0.5,3.3);


//       robot.TurningWithGyro(-90);

//        robot.TurningWithGyro(90);
//
//        robot.leftFrontDrive.setPower(0);
//       robot.leftBackDrive.setPower(0);
//        robot.rightFrontDrive.setPower(0);
//        robot.rightBackDrive.setPower(0);
//
//        robot.FollowingAngle(90, 0.5, 0.7);
    }


}
