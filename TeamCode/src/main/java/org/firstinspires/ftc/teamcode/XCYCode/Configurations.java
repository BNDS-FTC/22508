package org.firstinspires.ftc.teamcode.XCYCode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

@Config
public class Configurations {

    public static final class HardwareConstant {
        public static final String imuName = "imu";
        public static final String imu2Name = "imu2";
        public static final String leftFrontName = "lf";
        public static final DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
        public static final String leftBackName = "lb";
        public static final DcMotorSimple.Direction leftBackDirection = DcMotorSimple.Direction.REVERSE;
        public static final String rightBackName = "rb";
        public static final DcMotorSimple.Direction rightBackDirection = DcMotorSimple.Direction.FORWARD;
        public static final String rightFrontName = "rf";
        public static final DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;

        public static final String leftEncoderName = "lf";
        public static final Encoder.Direction leftEncoderDirection = Encoder.Direction.REVERSE;
        public static final String rightEncoderName = "rf";
        public static final Encoder.Direction rightEncoderDirection = Encoder.Direction.FORWARD;
        public static final String frontEncoderName = "lb";
        public static final Encoder.Direction frontEncoderDirection = Encoder.Direction.REVERSE;

        public static final String liftMotor = "lift";
        public static final String liftServo = "liftServo";
        public static final String spin = "spin";
        public static final String intakeMove0 = "intakeMove0";
        public static final String intakeMove1 = "intakeMove1";
        public static final DcMotorSimple.Direction intakeMove0Direction = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction intakeMove1Direction = DcMotorSimple.Direction.FORWARD;
        public static final String intakeHand0 = "intake2";
        public static final String intakeHand1 = "intake3";
        public static final String intakeArm = "intake1";
        public static final String doorServo = "door";
    }

    public static final double TICKS_PER_REV = 1;
    public static final double WHEEL_RADIUS = 50.8; // mm
    public static final double TRACK_WIDTH = 282; // mm
    public static final double WHEEL_BASE = 264; // mm

//    public static final double MAX_VEL = 1600;                        // 底盘速度
//    public static final double MAX_ACCEL = 3200;
//    public static final double MAX_ANG_VEL = Math.toRadians(200);
//    public static final double MAX_ANG_ACCEL = Math.toRadians(400);

    public static final double MAX_VEL = 1300;                        // 底盘速度
    public static final double MAX_ACCEL = 1800;
    public static final double MAX_ANG_VEL = Math.toRadians(150);
    public static final double MAX_ANG_ACCEL = Math.toRadians(150);

    public static final double kA = 0.00008;
    public static final double kStatic = 0.03;
    public static final double kV = 0.0004;

    public static double COLOR_DETECTION = 200;

    public static double encoderTicksToMM(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static final double LIFT_ARM_POS_EJECT = 0.86;                             //扣杆
    public static double LIFT_ARM_POS_TRANS = 0.09;                       //交接前低位
    public static double LIFT_ARM_POS_HOLD = 0.13;                              //交接抬手位
    public static final double LIFT_ARM_POS_VERTICAL = 0.65;
    public final static double LIFT_ARM_POS_LEVEL = 0.28;
    public final static double LIFT_ARM_POS_EJECT_MIN = 0.78;
    public final static double LIFT_ARM_POS_MAX = 0.88;

    public static final int LIFT_POS_MAX = 800;                                 //抬升滑轨

    public static int LIFT_POS_MIN = 0;

    public static double INTAKE_ARM_POS_LEVEL = 0.9;                           //小臂
    public static final double INTAKE_ARM_POS_AWAIT_TRANS = 0.4;
    public static final double INTAKE_ARM_POS_VERTICAL = 0.44;
    public static double INTAKE_ARM_POS_TRANS = 0.3;
    public static double INTAKE_ARM_POS_GROUND = 0.865;
    public static final double INTAKE_ARM_POS_MIN = 0.5; //TODO
    public static final double INTAKE_ARM_POS_LOW_JUNCTION = 0.565;
    public static final double[] INTAKE_ARM_POSITIONS = {0.89, 0.87, 0.84, 0.8, 0.76};
    public static final int[] TRANS_LIFT_POS_ADD = new int[]{0, 0, 0, 0, 0};

    public static final double INTAKE_HAND_0_POS = 0.55;                         //手(30kg)
    public static final double INTAKE_HAND_1_POS = 0.5;
    public static double INTAKE_HAND_GRIP = -0.185;

    public static final boolean protection_mode = false;

//    public static final double INTAKE_HAND_0_POS = 0.1;                       //手(快速)
//    public static final double INTAKE_HAND_1_POS = 0.9;

//    public static double INTAKE_HAND_0_POS = 0.57;                            //手(70kg)
//    public static double INTAKE_HAND_1_POS = 0.39;
//    public static double INTAKE_HAND_GRIP = -0.2;

    public static final double DOOR_HOLD = 0.83;
    public static final double DOOR_RELEASE = 0;

    public static final int INTAKE_MOVE_MAX_POS = 600;

    public static final int INTAKE_MOVE_WAY_POS = 1;

    public static final int SPIN_AMPLITUDE = 410;
    public static final int SPIN_MID = 0;

    public static void save_pos_in_csv(Pose2d pos) {
        try {
            String path = AppUtil.ROOT_FOLDER + "/RoadRunner/position.csv";
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(path), StandardCharsets.UTF_8));
            out.write(pos.getX() + "," + pos.getY() + "," + pos.getHeading() + ",");
            out.flush();
            out.close();
        } catch (Exception ignored) {
        }
    }

    @NonNull
    public static Pose2d get_pos_from_csv() {
        try {
            String path = AppUtil.ROOT_FOLDER + "/RoadRunner/position.csv";
            BufferedReader in = new BufferedReader(new InputStreamReader(new FileInputStream(path), StandardCharsets.UTF_8));
            String[] line = in.readLine().split(",");
            in.close();
            return new Pose2d(
                    Double.parseDouble(line[0]),
                    Double.parseDouble(line[1]),
                    Double.parseDouble(line[2])
            );
        } catch (Exception ignored) {
            return new Pose2d(-1, -1, -1);
        }
    }

    public static double clamp(double val, double limit) {
        return Range.clip(val, -limit, limit);
    }

    public static double gridlize(double val, double step, double add) {
        return (Math.round(val + add) / step) * step + add;
    }

    public static double map(double x, double a, double b, double c, double d) {
        if (x <= a) return c;
        else if (x >= b) return d;
        return ((x - a) / (b - a)) * (d - c) + c;
    }

    public static double map1(double x, double a, double b, double c, double d) {
        return ((x - a) / (b - a)) * (d - c) + c;
    }

    public  static class TimeoutException extends InterruptedException{
        public TimeoutException(){
            super();
        }
    }

    public static class StructureJamException extends InterruptedException{
        public StructureJamException(){
            super();
        }
    }

    public static class CollisionException extends InterruptedException{
        public CollisionException(){
            super();
        }
    }

    public static class ConeException extends InterruptedException{
        public ConeException(){
            super();
        }
    }

    public static class GlobalTimeoutException extends InterruptedException{
        public GlobalTimeoutException(){
            super();
        }
    }
}
