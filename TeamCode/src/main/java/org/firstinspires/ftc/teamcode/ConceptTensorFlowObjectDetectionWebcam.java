package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.onbotjava.RequestConditions;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@TeleOp(group = "Concept", name = "Teyanna Concept: TensorFlow Object Detection Webcam")
public class ConceptTensorFlowObjectDetectionWebcam extends LinearOpMode {
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY = "AcV6VDT/////AAABmcDT+7TuykpEi+TIsOYn69p/EevAM6yh00XwRvL6diJvpu4X7M5ZEyFYJRPGfRQh+XUbxMbRCDj0DyOKZES7dzrNHs74HbztGkTDqg6fR1KxN9w1dMa3kbBMhn9VYNc8fB0aog/GTA+T39IofjSTM59MX0Fm7uFFeYHSywHX7CVcAxmUHeqDRnoWWcSodWRLGS+l104zXvk2+1byTk6MrNIYki6sbHd2k5GnpsdxRliEG6nxvRnYdYMMrFqVO27D4koup2K+3WqLMcHG2Bx4Q9LSFayLnvgY2+Zb7IF8COZgCE/eIVpcqB9SdJT7KGtuKdJ1/TzozakHr0xGKyzSuQBEfvIbuYAhqCN64OcOym++";
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    public void runOpMode() {
        List<Recognition> updatedRecognitions;
        initVuforia();
        initTfod();
        TFObjectDetector tFObjectDetector = this.tfod;
        if (tFObjectDetector != null) {
            tFObjectDetector.activate();
            this.tfod.setZoom(1.25d, 1.78d);
        }
        this.telemetry.addData(">", (Object) "Press Play to start op mode");
        this.telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                TFObjectDetector tFObjectDetector2 = this.tfod;
                if (!(tFObjectDetector2 == null || (updatedRecognitions = tFObjectDetector2.getUpdatedRecognitions()) == null)) {
                    this.telemetry.addData("# Object Detected", (Object) Integer.valueOf(updatedRecognitions.size()));
                    for (Recognition recognition : updatedRecognitions) {
                        this.telemetry.addData(String.format("label (%d)", new Object[]{0}), (Object) recognition.getLabel());
                        this.telemetry.addData(String.format("  left,top (%d)", new Object[]{0}), "%.03f , %.03f", Float.valueOf(recognition.getLeft()), Float.valueOf(recognition.getTop()));
                        this.telemetry.addData(String.format("  right,bottom (%d)", new Object[]{0}), "%.03f , %.03f", Float.valueOf(recognition.getRight()), Float.valueOf(recognition.getBottom()));
                    }
                    this.telemetry.update();
                }
            }
        }
        TFObjectDetector tFObjectDetector3 = this.tfod;
        if (tFObjectDetector3 != null) {
            tFObjectDetector3.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = (CameraName) this.hardwareMap.get(WebcamName.class, "Webcam 1");
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(this.hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", RequestConditions.REQUEST_KEY_ID, this.hardwareMap.appContext.getPackageName()));
        tfodParameters.minResultConfidence = 0.8f;
        TFObjectDetector createTFObjectDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.vuforia);
        this.tfod = createTFObjectDetector;
        createTFObjectDetector.loadModelFromAsset("UltimateGoal.tflite", LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
