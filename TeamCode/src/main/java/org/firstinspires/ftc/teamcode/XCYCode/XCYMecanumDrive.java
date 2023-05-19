package org.firstinspires.ftc.teamcode.XCYCode;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class XCYMecanumDrive extends MecanumDrive {
    public final static PIDCoefficients AXIAL_PID = new PIDCoefficients(15, 0, 0.001);
    public final static PIDCoefficients LATERAL_PID = new PIDCoefficients(15, 0, 0.001);
    public final static PIDCoefficients HEADING_PID = new PIDCoefficients(9, 0.01, 0);

    public static final double LATERAL_MULTIPLIER = 1.15; //

    public static final double VX_WEIGHT = 1;
    public static final double VY_WEIGHT = 1.4;
    public static final double OMEGA_WEIGHT = 1;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH, WHEEL_BASE);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private final DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private final List<DcMotorEx> motors;

    private final BNO055IMU imu;
    private final VoltageSensor batteryVoltageSensor;

    public XCYMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, WHEEL_BASE, LATERAL_MULTIPLIER);

        TrajectoryFollower follower = new HolonomicPIDVAFollower(AXIAL_PID, LATERAL_PID, HEADING_PID,
                new Pose2d(70, 70, Math.toRadians(5)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
///        String filename = "22508controlHubIMUCalibration.json";
//        File file = AppUtil.getInstance().getSettingsFile(filename);
        imu = hardwareMap.get(BNO055IMU.class, Configurations.HardwareConstant.imuName);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.NORMAL;
//        parameters.calibrationData = new BNO055IMU.CalibrationData.deserialize(file);
        imu.initialize(parameters);

        setLocalizer(new XCYLocalizer(hardwareMap, imu));
///        setLocalizer(new StandardThreeWheelLocalizer(hardwareMap));

//        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
        leftFront = hardwareMap.get(DcMotorEx.class, HardwareConstant.leftFrontName);
        leftBack = hardwareMap.get(DcMotorEx.class, HardwareConstant.leftBackName);
        rightBack = hardwareMap.get(DcMotorEx.class, HardwareConstant.rightBackName);
        rightFront = hardwareMap.get(DcMotorEx.class, HardwareConstant.rightFrontName);

        leftFront.setDirection(HardwareConstant.leftFrontDirection);
        leftBack.setDirection(HardwareConstant.leftBackDirection);
        rightBack.setDirection(HardwareConstant.rightBackDirection);
        rightFront.setDirection(HardwareConstant.rightFrontDirection);
        motors = Arrays.asList(leftFront, leftBack, rightBack, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void stopTrajectory() {
        trajectorySequenceRunner.followTrajectorySequenceAsync(null);
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();

        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) {
            if (protection_mode && (getLastError().vec().norm() > 200 || Math.abs(getLastError().getHeading()) > Math.PI / 6)) {
                setDrivePower(new Pose2d(0, 0, 0));
                throw new RuntimeException("检查电机接线和外置编码器接线");
            } else {
                setDriveSignal(signal);
            }
        }
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public double getVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    private static final double DEAD_BAND = 0.001;

    public void setGlobalPower(Pose2d drivePower, double x_static, double y_static) {
        Vector2d vec = drivePower.vec().rotated(-getLocalizer().getPoseEstimate().getHeading());
        if (vec.norm() > DEAD_BAND) {
            vec = new Vector2d(
                    vec.getX() + Math.copySign(x_static, vec.getX()),
                    vec.getY() + Math.copySign(y_static, vec.getY())
            );
        }
        setWeightedDrivePower(new Pose2d(vec, drivePower.getHeading()));
    }

    public void setGlobalPower(Pose2d drivePower) {
        setGlobalPower(drivePower, 0, 0);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public void setPowerWithImu(Vector2d power, double heading_deg, double power_ratio) {
        double div = clamp(AngleUnit.normalizeDegrees(heading_deg - Math.toDegrees(getPoseEstimate().getHeading())) / 20, 1) * power_ratio;
        setWeightedDrivePower(new Pose2d(power, div));
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToMM(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToMM(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftBack.setPower(v1);
        rightBack.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth, double wheelBase) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth, wheelBase)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    public static PIDCoefficients translationPid = new PIDCoefficients(0.0065, 0.00025, 0.0009);
    public static PIDCoefficients headingPid = new PIDCoefficients(1.7, 0.25, 0.13);

    private PIDFController transPID_x;
    private PIDFController transPID_y;
    private PIDFController turnPID;
    private double moveHeading = 0;

    public void initSimpleMove(Pose2d pos) {
        stopTrajectory();
        transPID_x = new PIDFController(translationPid);
        transPID_x.setTargetPosition(pos.getX());

        transPID_y = new PIDFController(translationPid);
        transPID_y.setTargetPosition(pos.getY());

        turnPID = new PIDFController(headingPid);
        moveHeading = pos.getHeading();
        turnPID.setTargetPosition(0);
    }

    public void simpleMovePeriod() {
        simpleMovePeriod(0.95);
    }

    public void setSimpleMovePosition(Pose2d pos){
        transPID_x.setTargetPosition(pos.getX());
        transPID_y.setTargetPosition(pos.getY());
        moveHeading = pos.getHeading();
    }

    public Pose2d getSimpleMovePosition(){
        return new Pose2d(transPID_x.getTargetPosition(),transPID_y.getTargetPosition(), moveHeading);
    }

    public void simpleMovePeriod(double pow) {
        this.update();
        Pose2d current_pos = getPoseEstimate();
        this.setGlobalPower(new Pose2d(
                clamp(transPID_x.update(current_pos.getX()), pow),
                clamp(transPID_y.update(current_pos.getY()), pow),
                clamp(turnPID.update(AngleUnit.normalizeRadians(current_pos.getHeading() - moveHeading)), pow)
        ));
    }

    public boolean isTilt(){
        Orientation orientation = imu.getAngularOrientation();
        return Math.abs(orientation.secondAngle)>Math.toRadians(10)||Math.abs(orientation.thirdAngle)>Math.toRadians(10);
    }
}
