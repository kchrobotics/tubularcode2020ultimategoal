package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class BNO055IMUUtil {
    public static void remapAxes(BNO055IMU imu, AxesOrder order, AxesSigns signs) {
        try {
            int[] indices = order.indices();
            int axisMapConfig = 0 | (indices[0] << 4) | (indices[1] << 2) | (indices[2] << 0);
            int axisMapSign = (4 >> indices[0]) ^ signs.bVal;
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 15);
            Thread.sleep(100);
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfig & 63);
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, axisMapSign & 7);
            imu.write8(BNO055IMU.Register.OPR_MODE, imu.getParameters().mode.bVal & 15);
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
