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

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AcV6VDT/////AAABmcDT+7TuykpEi+TIsOYn69p/EevAM6yh00XwRvL6diJvpu4X7M5ZEyFYJRPGfRQh+XUbxMbRCDj0DyOKZES7dzrNHs74HbztGkTDqg6fR1KxN9w1dMa3kbBMhn9VYNc8fB0aog/GTA+T39IofjSTM59MX0Fm7uFFeYHSywHX7CVcAxmUHeqDRnoWWcSodWRLGS+l104zXvk2+1byTk6MrNIYki6sbHd2k5GnpsdxRliEG6nxvRnYdYMMrFqVO27D4koup2K+3WqLMcHG2Bx4Q9LSFayLnvgY2+Zb7IF8COZgCE/eIVpcqB9SdJT7KGtuKdJ1/TzozakHr0xGKyzSuQBEfvIbuYAhqCN64OcOym++";

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /* Public OpMode members. */
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor shooter = null;
    public DcMotor intake = null;
    public DcMotor lift = null;
    public Servo grabberservo = null;

    ModernRoboticsI2cRangeSensor rangeSensor;

    public BNO055IMU imu;
    Orientation angles;

    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot() {

    }
    LinearOpMode lopmode;
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode lopmode) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.lopmode= lopmode;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "lf");
        this.leftBackDrive = hwMap.get(DcMotor.class, "lb");
        rightFrontDrive = hwMap.get(DcMotor.class, "rf");
        rightBackDrive = hwMap.get(DcMotor.class, "rb");
        shooter = hwMap.get(DcMotor.class, "shooter");
        intake = hwMap.get(DcMotor.class, "intake");
        lift = hwMap.get(DcMotor.class, "lift");
        grabberservo =hwMap.get(Servo.class, "grabberservo");
        this.leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //reverse shooter if it doesn't work
        //reverse intake if it doesn't work
        //reverse lift if it doesn't work
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabberservo.setPosition(0.5);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*lopmode.telemetry.addData("Init", "BEFORE initvuforia");    //
        lopmode.telemetry.update();
        initVuforia();
        lopmode.telemetry.addData("Init", "after initvuforia");    //
        lopmode.telemetry.update();
        initTfod();
        lopmode.telemetry.addData("Init", "after initTfod");    //
        lopmode.telemetry.update();
        if (tfod != null) {
            tfod.activate();
            lopmode.telemetry.addData("Init", "tfod.activate");    //
            lopmode.telemetry.update();
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        else{
            lopmode.telemetry.addData("Init", "tfod.sad");    //
            lopmode.telemetry.update();
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
*/
    }

    public void RangeFinderDrive(double speed, double distance_cm) {

        leftFrontDrive.setPower(speed); //-
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        while (rangeSensor.getDistance(DistanceUnit.CM) > distance_cm) {
        }

        leftFrontDrive.setPower(0); //-
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setPower(-1 * speed / 2); //-
        leftBackDrive.setPower(-1 * speed / 2);
        rightFrontDrive.setPower(-1 * speed / 2);
        rightBackDrive.setPower(-1 * speed / 2);

        while (rangeSensor.getDistance(DistanceUnit.CM) < distance_cm) {
        }

        leftFrontDrive.setPower(0); //-
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

    }


    public void FollowingAngle(double targetangle, double speed, double time_s) {
        double error = 500;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (runtime.seconds() < time_s) {
            double differential = speed / 4;


            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = targetangle - FixAngle();

            int Sign = 1;

            if (error > 0) {
                Sign = 1;
            } else {
                Sign = -1;
            }

            differential = differential * Math.abs(error / 10);

            double leftspeed, rightspeed;
            leftspeed = speed - (Sign * differential);
            rightspeed = speed + (Sign * differential);

            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > 1.0) {
                leftspeed /= max;
                rightspeed /= max;
            }

            leftFrontDrive.setPower(leftspeed); //-
            leftBackDrive.setPower(leftspeed);
            rightFrontDrive.setPower(rightspeed);
            rightBackDrive.setPower(rightspeed);
        }
    }

    public void FollowingAngleDistance(double targetangle, double speed, double distanceaway_cm) {
        double error = 500;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (rangeSensor.getDistance(DistanceUnit.CM) > distanceaway_cm) {
            double differential = speed / 4;


            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = targetangle - FixAngle();

            int Sign = 1;

            if (error > 0) {
                Sign = 1;
            } else {
                Sign = -1;
            }

            differential = differential * Math.abs(error / 10);

            double leftspeed, rightspeed;
            leftspeed = speed - (Sign * differential);
            rightspeed = speed + (Sign * differential);

            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > 1.0) {
                leftspeed /= max;
                rightspeed /= max;
            }

            leftFrontDrive.setPower(leftspeed); //-
            leftBackDrive.setPower(leftspeed);
            rightFrontDrive.setPower(rightspeed);
            rightBackDrive.setPower(rightspeed);
        }
    }

    public void FollowingAngleDistanceAway(double targetangle, double speed, double distanceaway_cm) {
        double error = 500;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (rangeSensor.getDistance(DistanceUnit.CM) < distanceaway_cm) {
            double differential = speed / 4;


            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = targetangle - FixAngle();

            int Sign = 1;

            if (error > 0) {
                Sign = 1;
            } else {
                Sign = -1;
            }

            differential = differential * Math.abs(error / 10);

            double leftspeed, rightspeed;
            leftspeed = speed - (Sign * differential);
            rightspeed = speed + (Sign * differential);

            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > 1.0) {
                leftspeed /= max;
                rightspeed /= max;
            }

            leftFrontDrive.setPower(leftspeed); //-
            leftBackDrive.setPower(leftspeed);
            rightFrontDrive.setPower(rightspeed);
            rightBackDrive.setPower(rightspeed);
        }
    }

    double OldAngle = 0;
    double FixValue = 0;


    public float FixAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (OldAngle - angles.firstAngle > 180) {
            FixValue = FixValue + 360;
        } else if (OldAngle - angles.firstAngle < -180) {
            FixValue = FixValue - 360;

        }
        OldAngle = angles.firstAngle;
        return (float) (OldAngle + FixValue);


    }

    public void TurningWithGyro(double targetangle) {
        double error = 500;
        while (Math.abs(error) > 2) {

            //angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = targetangle - FixAngle();

            float Sign = 1;

            if (error > 0) {
                Sign = 1;
            } else {
                Sign = -1;
            }
            if (error > 20) {
                Sign = Sign * 1.5f;
            }
            leftFrontDrive.setPower(-0.2 * Sign);
            leftBackDrive.setPower(-0.2 * Sign);
            rightFrontDrive.setPower(0.2 * Sign);
            rightBackDrive.setPower(0.2 * Sign);
        }
    }

    public int highrings() {

        //More time in autonomous if needed

        if (tfod == null) {
            return -10;
        }
        int result = -99;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            result = -98;
            lopmode.telemetry.addData("Init", "updatedRecognition");    //
            lopmode.telemetry.update();
            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.

                for (int i = 0; i < 100; i++) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        lopmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.

                        if (updatedRecognitions.size() < 1)
                            result = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            lopmode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                result = 4;
                            } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                result = 1;
                            } else {
                                result = 0;
                            }
                            lopmode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            lopmode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                        }
                        lopmode.telemetry.update();
                    }
                }
            }
            lopmode.telemetry.addData("Init", "Done with loop");    //
            lopmode.telemetry.update();


        }
        Log.d("high","problem");
        return result;
    }

    public void liftPosition(boolean up){

        if (up){
           lift.setTargetPosition(1000);
           lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           lift.setPower(1);
        } else
        {
            lift.setTargetPosition(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
        }

    }

    private void initTfod() {

        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;


        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

       // lopmode.telemetry.update();
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
}

