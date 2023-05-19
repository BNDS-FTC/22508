package org.firstinspires.ftc.teamcode.XCYCode;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.LIFT_ARM_POS_EJECT_MIN;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.LIFT_ARM_POS_MAX;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.SPIN_AMPLITUDE;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.clamp;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.map;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.function.BooleanSupplier;

public class WebcamTeleOpSuperStructure extends TeleOpSuperStructure {
    private final boolean DEBUG = true;

    private final OpenCvWebcam webcam;
    private final XCYJunctionAimPipeline pipeline;
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;
    public static int WHITE_BALANCE = 5200;

    public WebcamTeleOpSuperStructure(HardwareMap hardwareMap, BooleanSupplier continueCondition, Runnable drivePeriod) {
        super(hardwareMap, continueCondition, drivePeriod);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new XCYJunctionAimPipeline();
        pipeline.setDEBUG(DEBUG);
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
        if (DEBUG)
            FtcDashboard.getInstance().startCameraStream(webcam, 10);
    }

    public void setWebcamConfig() {
        webcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
        sleep_with_drive(500);
//        int whiteBalance = webcam.getWhiteBalanceControl().getWhiteBalanceTemperature();
//        webcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
//        webcam.getWhiteBalanceControl().setWhiteBalanceTemperature(whiteBalance);

        webcam.getExposureControl().setMode(ExposureControl.Mode.ContinuousAuto);
//        webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
//        webcam.getExposureControl().setExposure(5, TimeUnit.MILLISECONDS);
        webcam.getGainControl().setGain(0);
    }

    public static PIDCoefficients spinAimPIDCoefficients = new PIDCoefficients(0.003, 0, 0.0001);
    private final PIDFController spinAimPID = new PIDFController(spinAimPIDCoefficients);

    public void junctionAim() {
        double power = clamp(-spinAimPID.update(getJunctionOffset()), 0.4);
        int pos = spinBase.getCurrentPosition();
        if (webcamIsDetected()) {
            if ((pos < -SPIN_AMPLITUDE && power < 0) || (pos > SPIN_AMPLITUDE && power > 0)) {
                setSpinPosition(0);
                sleep_with_drive(400);
            } else {
                spinBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                spinBase.setPower(power);
            }
        } else {
            setSpinPosition(0);
        }
    }

    private XCYBoolean[] aimReleaseCondition = {new XCYBoolean(() -> false)};

    public void setAimReleaseCondition(XCYBoolean... condition) {
        aimReleaseCondition = condition;
    }

    @Override
    public void toMidJunction() {
        super.toMidJunction();
        while (continue_condition.getAsBoolean()) {
            drive_period.run();
            for (XCYBoolean b : aimReleaseCondition) {
                if (b.toTrue()) return;
            }
            double distance = pipeline.getJunctionDistance();
            junctionAim();
            if (distance > MID_DISTANCE_MID) {
                liftArmServo.setPosition(LIFT_ARM_POS_MAX);
                setLifter((int) map(
                                distance,
                                MID_DISTANCE_MID,
                                MID_DISTANCE_MAX,
                                MID_LIFT_MID,
                                MID_LIFT_MAX),
                        0.6);
            } else {
                liftArmServo.setPosition(map(
                        distance,
                        MID_DISTANCE_MIN,
                        MID_DISTANCE_MID,
                        LIFT_ARM_POS_EJECT_MIN,
                        LIFT_ARM_POS_MAX
                ));
                setLifter((int) map(
                                distance,
                                MID_DISTANCE_MIN,
                                MID_DISTANCE_MID,
                                MID_LIFT_MIN,
                                MID_LIFT_MID),
                        0.6);
            }
        }
    }

    @Override
    public void toLowJunction() {
        super.toLowJunction();
        while (continue_condition.getAsBoolean()) {
            drive_period.run();
            for (XCYBoolean b : aimReleaseCondition) {
                if (b.toTrue()) return;
            }
            junctionAim();
            double distance = pipeline.getJunctionDistance();
            liftArmServo.setPosition(LIFT_ARM_POS_MAX);
            setLifter((int) map(
                            distance,
                            LOW_DISTANCE_MIN,
                            LOW_DISTANCE_MAX,
                            LOW_LIFT_MIN,
                            LOW_LIFT_MAX),
                    0.6);

        }
    }

    @Override
    public void toHighJunction() {
        super.toHighJunction();
        while (isBusy() && continue_condition.getAsBoolean()) {
            drive_period.run();
        }
        while (continue_condition.getAsBoolean()) {
            drive_period.run();
            for (XCYBoolean b : aimReleaseCondition) {
                if (b.toTrue()) return;
            }
            double distance = pipeline.getJunctionDistance();
            junctionAim();
            if (distance > HIGH_DISTANCE_MID) {
                liftArmServo.setPosition(LIFT_ARM_POS_MAX);
                setLifter((int) map(
                                distance,
                                HIGH_DISTANCE_MID,
                                HIGH_DISTANCE_MAX,
                                HIGH_LIFT_MID,
                                HIGH_LIFT_MAX),
                        0.6);
            } else {
                liftArmServo.setPosition(map(
                        distance,
                        HIGH_DISTANCE_MIN,
                        HIGH_DISTANCE_MID,
                        LIFT_ARM_POS_EJECT_MIN,
                        LIFT_ARM_POS_MAX
                ));
                setLifter((int) map(
                                distance,
                                HIGH_DISTANCE_MIN,
                                HIGH_DISTANCE_MID,
                                HIGH_LIFT_MIN,
                                HIGH_LIFT_MID),
                        0.6);
            }
        }
    }

    public double getJunctionOffset() {
        return pipeline.getJunctionOffset();
    }

    public double getJunctionDistance() {
        return pipeline.getJunctionDistance();
    }

    public boolean webcamIsDetected() {
        return pipeline.isDetected();
    }

    public int getElapseTime() {
        return webcam.getPipelineTimeMs();
    }
}
