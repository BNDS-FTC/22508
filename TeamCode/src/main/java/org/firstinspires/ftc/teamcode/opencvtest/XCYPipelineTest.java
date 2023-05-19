package org.firstinspires.ftc.teamcode.opencvtest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.XCYCode.XCYJunctionAimPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class XCYPipelineTest extends LinearOpMode {
    public static int exposure_time = 5;
    public static int gain = 0;
    public static int wb = 4200;

    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;

    OpenCvWebcam webcam;
    XCYJunctionAimPipeline pipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new XCYJunctionAimPipeline();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();

        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        waitForStart();
//        webcam.getExposureControl().setMode(ExposureControl.Mode.ContinuousAuto);
        webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);

        webcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
        int last_gain = gain;
        int last_exp_time = exposure_time;
        /*
         * exposure: 15
         * max exposure: 500
         * min exposure: 0
         * max gian: 100
         * min gian: 0
         * wb max: 6500
         * wb min: 2800
         */
        while (opModeIsActive()) {
            telemetry.addData("offset", pipeline.getJunctionOffset());
            telemetry.addData("distance",pipeline.getJunctionDistance());
            telemetry.addData("wb max", webcam.getWhiteBalanceControl().getMaxWhiteBalanceTemperature());
            telemetry.addData("wb min", webcam.getWhiteBalanceControl().getMinWhiteBalanceTemperature());
            telemetry.addData("wb", webcam.getWhiteBalanceControl().getWhiteBalanceTemperature());
//            telemetry.addData("lift", liftMotor.getCurrentPosition());
            telemetry.update();
            webcam.getGainControl().setGain(gain);
            webcam.getExposureControl().setExposure(exposure_time, TimeUnit.MILLISECONDS);
            webcam.getWhiteBalanceControl().setWhiteBalanceTemperature(wb);
            sleep(10);
            last_gain = gain;
            last_exp_time = exposure_time;
        }
    }
}
