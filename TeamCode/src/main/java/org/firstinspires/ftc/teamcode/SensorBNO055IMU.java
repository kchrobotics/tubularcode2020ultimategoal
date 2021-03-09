package org.firstinspires.ftc.teamcode;

import androidx.core.app.NotificationCompat;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(group = "Sensor", name = "Teyanna: BNO055 IMU")
public class SensorBNO055IMU extends LinearOpMode {
    Orientation angles;
    Acceleration gravity;
    BNO055IMU imu;

    public void runOpMode() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        BNO055IMU bno055imu = (BNO055IMU) this.hardwareMap.get(BNO055IMU.class, "imu");
        this.imu = bno055imu;
        bno055imu.initialize(parameters);
        composeTelemetry();
        waitForStart();
        this.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {
            this.telemetry.update();
        }
    }

    /* access modifiers changed from: package-private */
    public void composeTelemetry() {
        this.telemetry.addAction(new Runnable() {
            public void run() {
                SensorBNO055IMU sensorBNO055IMU = SensorBNO055IMU.this;
                sensorBNO055IMU.angles = sensorBNO055IMU.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                SensorBNO055IMU sensorBNO055IMU2 = SensorBNO055IMU.this;
                sensorBNO055IMU2.gravity = sensorBNO055IMU2.imu.getGravity();
            }
        });
        this.telemetry.addLine().addData(NotificationCompat.CATEGORY_STATUS, new Func<String>() {
            public String value() {
                return SensorBNO055IMU.this.imu.getSystemStatus().toShortString();
            }
        }).addData("calib", new Func<String>() {
            public String value() {
                return SensorBNO055IMU.this.imu.getCalibrationStatus().toString();
            }
        });
        this.telemetry.addLine().addData("heading", new Func<String>() {
            public String value() {
                SensorBNO055IMU sensorBNO055IMU = SensorBNO055IMU.this;
                return sensorBNO055IMU.formatAngle(sensorBNO055IMU.angles.angleUnit, (double) SensorBNO055IMU.this.angles.firstAngle);
            }
        }).addData("roll", new Func<String>() {
            public String value() {
                SensorBNO055IMU sensorBNO055IMU = SensorBNO055IMU.this;
                return sensorBNO055IMU.formatAngle(sensorBNO055IMU.angles.angleUnit, (double) SensorBNO055IMU.this.angles.secondAngle);
            }
        }).addData("pitch", new Func<String>() {
            public String value() {
                SensorBNO055IMU sensorBNO055IMU = SensorBNO055IMU.this;
                return sensorBNO055IMU.formatAngle(sensorBNO055IMU.angles.angleUnit, (double) SensorBNO055IMU.this.angles.thirdAngle);
            }
        });
        this.telemetry.addLine().addData("grvty", new Func<String>() {
            public String value() {
                return SensorBNO055IMU.this.gravity.toString();
            }
        }).addData("mag", new Func<String>() {
            public String value() {
                return String.format(Locale.getDefault(), "%.3f", new Object[]{Double.valueOf(Math.sqrt((SensorBNO055IMU.this.gravity.xAccel * SensorBNO055IMU.this.gravity.xAccel) + (SensorBNO055IMU.this.gravity.yAccel * SensorBNO055IMU.this.gravity.yAccel) + (SensorBNO055IMU.this.gravity.zAccel * SensorBNO055IMU.this.gravity.zAccel)))});
            }
        });
    }

    /* access modifiers changed from: package-private */
    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    /* access modifiers changed from: package-private */
    public String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", new Object[]{Double.valueOf(AngleUnit.DEGREES.normalize(degrees))});
    }
}
