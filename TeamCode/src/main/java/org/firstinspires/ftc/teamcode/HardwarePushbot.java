package org.firstinspires.ftc.teamcode;

import android.util.Log;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.onbotjava.RequestConditions;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class HardwarePushbot {
    public static final double ARM_DOWN_POWER = -0.45d;
    public static final double ARM_UP_POWER = 0.45d;
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public static final double MID_SERVO = 0.5d;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY = "AcV6VDT/////AAABmcDT+7TuykpEi+TIsOYn69p/EevAM6yh00XwRvL6diJvpu4X7M5ZEyFYJRPGfRQh+XUbxMbRCDj0DyOKZES7dzrNHs74HbztGkTDqg6fR1KxN9w1dMa3kbBMhn9VYNc8fB0aog/GTA+T39IofjSTM59MX0Fm7uFFeYHSywHX7CVcAxmUHeqDRnoWWcSodWRLGS+l104zXvk2+1byTk6MrNIYki6sbHd2k5GnpsdxRliEG6nxvRnYdYMMrFqVO27D4koup2K+3WqLMcHG2Bx4Q9LSFayLnvgY2+Zb7IF8COZgCE/eIVpcqB9SdJT7KGtuKdJ1/TzozakHr0xGKyzSuQBEfvIbuYAhqCN64OcOym++";
    double FixValue = LynxServoController.apiPositionFirst;
    double OldAngle = LynxServoController.apiPositionFirst;
    Orientation angles;
    public Servo dropperservo = null;
    public Servo grabberservo = null;
    HardwareMap hwMap = null;
    public BNO055IMU imu;
    public DcMotor intake = null;
    public DcMotor leftBackDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor lift = null;
    LinearOpMode lopmode;
    private ElapsedTime period = new ElapsedTime();
    public DcMotor rightBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DistanceSensor sensorRange;
    public DcMotor shooter = null;
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    public void init(HardwareMap ahwMap, LinearOpMode lopmode2, boolean usedvuforia) {
        this.hwMap = ahwMap;
        this.lopmode = lopmode2;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        BNO055IMU bno055imu = (BNO055IMU) this.hwMap.get(BNO055IMU.class, "imu");
        this.imu = bno055imu;
        bno055imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(this.imu, AxesOrder.XYZ, AxesSigns.NPN);
        this.leftFrontDrive = (DcMotor) this.hwMap.get(DcMotor.class, "lf");
        this.leftBackDrive = (DcMotor) this.hwMap.get(DcMotor.class, "lb");
        this.rightFrontDrive = (DcMotor) this.hwMap.get(DcMotor.class, "rf");
        this.rightBackDrive = (DcMotor) this.hwMap.get(DcMotor.class, "rb");
        this.shooter = (DcMotor) this.hwMap.get(DcMotor.class, "shooter");
        this.intake = (DcMotor) this.hwMap.get(DcMotor.class, "intake");
        this.lift = (DcMotor) this.hwMap.get(DcMotor.class, "lift");
        this.grabberservo = (Servo) this.hwMap.get(Servo.class, "grabberservo");
        this.dropperservo = (Servo) this.hwMap.get(Servo.class, "dropperservo");
        this.leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        DistanceSensor distanceSensor = (DistanceSensor) this.hwMap.get(DistanceSensor.class, "sensor_range");
        this.sensorRange = distanceSensor;
        Rev2mDistanceSensor rev2mDistanceSensor = (Rev2mDistanceSensor) distanceSensor;
        this.leftFrontDrive.setPower(LynxServoController.apiPositionFirst);
        this.leftBackDrive.setPower(LynxServoController.apiPositionFirst);
        this.rightFrontDrive.setPower(LynxServoController.apiPositionFirst);
        this.rightBackDrive.setPower(LynxServoController.apiPositionFirst);
        this.shooter.setPower(LynxServoController.apiPositionFirst);
        this.intake.setPower(LynxServoController.apiPositionFirst);
        this.lift.setPower(LynxServoController.apiPositionFirst);
        this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lift.setTargetPosition(0);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.grabberservo.setPosition(0.5d);
        this.dropperservo.setPosition(0.5d);
        this.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (usedvuforia) {
            lopmode2.telemetry.addData("Init", (Object) "BEFORE initvuforia");
            lopmode2.telemetry.update();
            initVuforia();
            lopmode2.telemetry.addData("Init", (Object) "after initvuforia");
            lopmode2.telemetry.update();
            initTfod();
            lopmode2.telemetry.addData("Init", (Object) "after initTfod");
            lopmode2.telemetry.update();
            TFObjectDetector tFObjectDetector = this.tfod;
            if (tFObjectDetector != null) {
                tFObjectDetector.activate();
                this.tfod.setZoom(1.25d, 1.78d);
                lopmode2.telemetry.addData("Init", (Object) "tfod.activate");
                lopmode2.telemetry.update();
                return;
            }
            lopmode2.telemetry.addData("Init", (Object) "tfod.sad");
            lopmode2.telemetry.update();
        }
    }

    public void RangeFinderDrive(double speed, double distance_cm) {
        this.leftFrontDrive.setPower(speed);
        this.leftBackDrive.setPower(speed);
        this.rightFrontDrive.setPower(speed);
        this.rightBackDrive.setPower(speed);
        do {
        } while (this.sensorRange.getDistance(DistanceUnit.CM) > distance_cm);
        this.leftFrontDrive.setPower(LynxServoController.apiPositionFirst);
        this.leftBackDrive.setPower(LynxServoController.apiPositionFirst);
        this.rightFrontDrive.setPower(LynxServoController.apiPositionFirst);
        this.rightBackDrive.setPower(LynxServoController.apiPositionFirst);
        this.leftFrontDrive.setPower((speed * -1.0d) / 2.0d);
        this.leftBackDrive.setPower((speed * -1.0d) / 2.0d);
        this.rightFrontDrive.setPower((speed * -1.0d) / 2.0d);
        this.rightBackDrive.setPower((-1.0d * speed) / 2.0d);
        do {
        } while (this.sensorRange.getDistance(DistanceUnit.CM) < distance_cm);
        this.leftFrontDrive.setPower(LynxServoController.apiPositionFirst);
        this.leftBackDrive.setPower(LynxServoController.apiPositionFirst);
        this.rightFrontDrive.setPower(LynxServoController.apiPositionFirst);
        this.rightBackDrive.setPower(LynxServoController.apiPositionFirst);
    }

    public void FollowingAngle(double targetangle, double speed, double time_s) {
        int Sign;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < time_s) {
            double differential = Math.abs(speed / 4.0d);
            double error = targetangle - ((double) FixAngle());
            if (error > LynxServoController.apiPositionFirst) {
                Sign = 1;
            } else {
                Sign = -1;
            }
            double differential2 = differential * Math.abs(error / 10.0d);
            double leftspeed = speed - (((double) Sign) * differential2);
            double rightspeed = speed + (((double) Sign) * differential2);
            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > 1.0d) {
                leftspeed /= max;
                rightspeed /= max;
            }
            this.leftFrontDrive.setPower(leftspeed);
            this.leftBackDrive.setPower(leftspeed);
            this.rightFrontDrive.setPower(rightspeed);
            this.rightBackDrive.setPower(rightspeed);
        }
    }

    public void FollowingAngleDistance(double targetangle, double speed, double distanceaway_cm) {
        int Sign;
        new ElapsedTime().reset();
        while (this.sensorRange.getDistance(DistanceUnit.CM) > distanceaway_cm) {
            double differential = Math.abs(speed / 4.0d);
            double error = targetangle - ((double) FixAngle());
            if (error > LynxServoController.apiPositionFirst) {
                Sign = 1;
            } else {
                Sign = -1;
            }
            double differential2 = differential * Math.abs(error / 10.0d);
            double leftspeed = speed - (((double) Sign) * differential2);
            double rightspeed = speed + (((double) Sign) * differential2);
            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > 1.0d) {
                leftspeed /= max;
                rightspeed /= max;
            }
            this.leftFrontDrive.setPower(leftspeed);
            this.leftBackDrive.setPower(leftspeed);
            this.rightFrontDrive.setPower(rightspeed);
            this.rightBackDrive.setPower(rightspeed);
        }
    }

    public void FollowingAngleDistanceAway(double targetangle, double speed, double distanceaway_cm) {
        int Sign;
        new ElapsedTime().reset();
        while (this.sensorRange.getDistance(DistanceUnit.CM) < distanceaway_cm) {
            double differential = Math.abs(speed / 4.0d);
            double error = targetangle - ((double) FixAngle());
            if (error > LynxServoController.apiPositionFirst) {
                Sign = 1;
            } else {
                Sign = -1;
            }
            double differential2 = differential * Math.abs(error / 10.0d);
            double leftspeed = speed - (((double) Sign) * differential2);
            double rightspeed = speed + (((double) Sign) * differential2);
            double max = Math.max(Math.abs(leftspeed), Math.abs(rightspeed));
            if (max > 1.0d) {
                leftspeed /= max;
                rightspeed /= max;
            }
            this.leftFrontDrive.setPower(leftspeed);
            this.leftBackDrive.setPower(leftspeed);
            this.rightFrontDrive.setPower(rightspeed);
            this.rightBackDrive.setPower(rightspeed);
        }
    }

    public float FixAngle() {
        Orientation angularOrientation = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.angles = angularOrientation;
        if (this.OldAngle - ((double) angularOrientation.firstAngle) > 180.0d) {
            this.FixValue += 360.0d;
        } else if (this.OldAngle - ((double) this.angles.firstAngle) < -180.0d) {
            this.FixValue -= 360.0d;
        }
        double d = (double) this.angles.firstAngle;
        this.OldAngle = d;
        return (float) (d + this.FixValue);
    }

    public void TurningWithGyro(double targetangle) {
        float Sign;
        double error = 500.0d;
        while (Math.abs(error) > 2.0d) {
            error = targetangle - ((double) FixAngle());
            if (error > LynxServoController.apiPositionFirst) {
                Sign = 1.0f;
            } else {
                Sign = -1.0f;
            }
            if (error > 20.0d) {
                Sign *= 1.5f;
            }
            this.leftFrontDrive.setPower(((double) Sign) * -0.3d);
            this.leftBackDrive.setPower(((double) Sign) * -0.3d);
            this.rightFrontDrive.setPower(((double) Sign) * 0.3d);
            this.rightBackDrive.setPower(((double) Sign) * 0.3d);
        }
    }

    public int highrings() {
        TFObjectDetector tFObjectDetector = this.tfod;
        if (tFObjectDetector == null) {
            return -10;
        }
        int i = -99;
        if (tFObjectDetector != null) {
            int result = -98;
            this.lopmode.telemetry.addData("Init", (Object) "updatedRecognition");
            this.lopmode.telemetry.update();
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (this.tfod != null) {
                for (int i2 = 0; i2 < 100; i2++) {
                    List<Recognition> updatedRecognitions = this.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        this.lopmode.telemetry.addData("# Object Detected", (Object) Integer.valueOf(updatedRecognitions.size()));
                        for (Recognition recognition : updatedRecognitions) {
                            this.lopmode.telemetry.addData(String.format("label (%d)", new Object[]{Integer.valueOf(i2)}), (Object) recognition.getLabel());
                            if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                                result = 4;
                            } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                result = 1;
                            } else {
                                result = 0;
                            }
                            this.lopmode.telemetry.addData(String.format("  left,top (%d)", new Object[]{Integer.valueOf(i2)}), "%.03f , %.03f,  %s", Float.valueOf(recognition.getLeft()), Float.valueOf(recognition.getTop()), recognition.getLabel());
                            this.lopmode.telemetry.addData(String.format("  right,bottom (%d)", new Object[]{Integer.valueOf(i2)}), "%.03f , %.03f", Float.valueOf(recognition.getRight()), Float.valueOf(recognition.getBottom()));
                        }
                        this.lopmode.telemetry.update();
                    }
                }
                i = result;
            } else {
                i = -98;
            }
            this.lopmode.telemetry.addData("Init", (Object) "Done with loop");
            this.lopmode.telemetry.update();
        }
        Log.d("high", "problem");
        return i;
    }

    public void liftPosition(boolean up) {
        if (up) {
            this.lift.setTargetPosition(1374);
            this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.lift.setPower(1.0d);
            return;
        }
        this.lift.setTargetPosition(100);
        this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.lift.setPower(1.0d);
    }

    private void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(this.hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", RequestConditions.REQUEST_KEY_ID, this.hwMap.appContext.getPackageName()));
        tfodParameters.minResultConfidence = 0.8f;
        TFObjectDetector createTFObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
        this.tfod = createTFObjectDetector;
        createTFObjectDetector.loadModelFromAsset("UltimateGoal.tflite", LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = (CameraName) this.hwMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}
