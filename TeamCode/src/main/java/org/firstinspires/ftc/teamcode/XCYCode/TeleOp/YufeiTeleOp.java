//package org.firstinspires.ftc.teamcode.XCYCode.TeleOp;
//
//
//import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;
//
//import android.os.Build;
//
//import androidx.annotation.RequiresApi;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.XCYCode.Configurations;
//import org.firstinspires.ftc.teamcode.XCYCode.XCYBoolean;
//import org.firstinspires.ftc.teamcode.XCYCode.XCYMecanumDrive;
//import org.firstinspires.ftc.teamcode.XCYCode.XCYSuperStructure;
//
//@TeleOp(name = "Yufei TeleOp")
//@Config
//public class YufeiTeleOp extends LinearOpMode {
//
//    private static final double max_turn_assist_power = 0.4;
//
//    private double global_drive_power = 1;
//    private double global_drive_turn_ratio = 1;
//    private XCYSuperStructure upper;
//
//    enum IntakeStatus {
//        EMPTY_HORIZONTAL, HOLDING_VERTICAL, HOLDING_HORIZONTAL, EMPTY_VERTICAL
//    }
//
//    public static double x_static_compensation = 0.0;
//    public static double y_static_compensation = 0.0;
//
//    IntakeStatus intake_status = IntakeStatus.EMPTY_VERTICAL;
//    XCYMecanumDrive drive;
//    private Pose2d current_pos;
//    private Vector2d cone_pos_vec = new Vector2d(50, -1200);
//    double current_intake_length;
//    private XCYBoolean dpad_down;
//    private XCYBoolean dpad_up;
//
//    /**
//     * left_stick/trigger: 移动 =<p>
//     * <p>
//     * a: 打开瞄准模式<p>
//     * b: 夹取舵机垂直 -- 降低舵机 -- 松手<p>
//     * x: 释放/回收<p>
//     * y: 高位<p>
//     * start: 刷分模式定位<p>
//     * dpad up: 中位<p>
//     * dpad down: 低位<p>
//     * bumper: 自动移动<p>
//     * 2right bumper: 刷分<p>
//     * dpad up: 向前<p>
//     * dpad down: 向后<p>
//     * 2left bumper: 暂停刷分<p>
//     */
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        upper = new XCYSuperStructure(
//                hardwareMap,
//                this::opModeIsActive,
//                () -> {
//                    drive_period();
//                    logic_period();
//                });
//        drive = new XCYMecanumDrive(hardwareMap);
//        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        drive.getLocalizer().setPoseEstimate(Configurations.get_pos_from_csv());
//        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color");
//        waitForStart();
//        current_intake_length = 420;
//
//        XCYBoolean color_detected = new XCYBoolean(() -> colorSensor.red() > COLOR_DETECTION);
//        XCYBoolean manual_aim_mode = new XCYBoolean(() -> gamepad1.a || gamepad2.a);
//        XCYBoolean intake_action = new XCYBoolean(() -> gamepad1.b || gamepad2.b);
//        XCYBoolean upper_release = new XCYBoolean(() -> gamepad1.x || gamepad2.x || gamepad1.left_stick_button);
//        XCYBoolean button_y = new XCYBoolean(() -> gamepad1.y || gamepad2.y);
//
//        dpad_down = new XCYBoolean(() -> gamepad1.dpad_down || gamepad2.dpad_down);
//        dpad_up = new XCYBoolean(() -> gamepad1.dpad_up || gamepad2.dpad_up);
//        XCYBoolean auto_aim_p2 = new XCYBoolean(() -> (gamepad1.left_bumper || gamepad1.right_stick_button) && intake_status != IntakeStatus.HOLDING_HORIZONTAL);
//        XCYBoolean auto_aim_p1 = new XCYBoolean(() -> gamepad1.right_bumper);
//        upper.set_eject_spin_pos(0);
//
//        boolean color_blind = true;
//        while (opModeIsActive()) {
//            logic_period();
//
//            if (auto_aim_p1.toTrue() && (intake_status == IntakeStatus.EMPTY_HORIZONTAL || intake_status == IntakeStatus.EMPTY_VERTICAL)) {
//                color_blind = true;
//                upper.toOriginal();
//                double end_pos_y = cone_pos_vec.getY();
//                double target_angle = Math.copySign(Math.PI / 2, AngleUnit.normalizeRadians(current_pos.getHeading()));
//                drive.initPreciseMove(new Pose2d(current_pos.getX(), end_pos_y, target_angle));
//                do {
//                    logic_period();
//                    drive.preciseMovePeriod();
//                } while (auto_aim_p1.get());
//                global_drive_power = 1;
//                global_drive_turn_ratio = 1;
//                intake_status = IntakeStatus.EMPTY_VERTICAL;
//            } else if (auto_aim_p2.toTrue() && !color_detected.get() && (intake_status == IntakeStatus.EMPTY_HORIZONTAL || intake_status == IntakeStatus.EMPTY_VERTICAL)) {
//                color_blind = false;
//                double pos_err = current_pos.vec().minus(cone_pos_vec).norm();
//                double intake_final_distance = Math.min(XCYSuperStructure.MAX_INTAKE_MOVE_DISTANCE, pos_err);
//                double target_angle = cone_pos_vec.minus(current_pos.vec()).angle();
//                Vector2d intake_vec = new Vector2d(intake_final_distance, 0).rotated(target_angle);
//                drive.initPreciseMove(new Pose2d(cone_pos_vec.minus(intake_vec), target_angle));
//                upper.toOriginal();
//                XCYBoolean aim_ready = new XCYBoolean(() -> Math.abs(AngleUnit.normalizeRadians(current_pos.getHeading() - target_angle)) < Math.toRadians(20));
//                if (aim_ready.get()) upper.toAim();
//                do {
//                    logic_period();
//                    drive.preciseMovePeriod();
//                    if (aim_ready.toTrue()) {
//                        upper.toAim();
//                        long time = System.currentTimeMillis();
//                        while (System.currentTimeMillis() - time < 200 && auto_aim_p2.get()) {
//                            logic_period();
//                            upper.setIntakeMoveLength(intake_final_distance);
//                            drive.preciseMovePeriod();
//                        }
//                    } else if (aim_ready.get()) {
//                        upper.setIntakeMoveLength(intake_final_distance);
//                    }
//                } while (auto_aim_p2.get() && !color_detected.get());
//                global_drive_power = 0.5;
//                global_drive_turn_ratio = 0.3;
//                upper.setIntakeMoveLength(upper.getIntakeMoveLength());
//                intake_status = IntakeStatus.EMPTY_HORIZONTAL;
//                aim_ready.deactivate();
//            } else {
//                drive_period();
//            }
//
//            if (manual_aim_mode.toTrue()) {
//                //瞄准模式
//                global_drive_power = 0.4;
//                global_drive_turn_ratio = 0.4;
//                upper.toAim();
//                upper.sleep_with_drive(300);
//                intake_status = IntakeStatus.EMPTY_HORIZONTAL;
//                color_blind = false;
//            }
//
//            if (intake_action.toTrue()) {
//                //b
//                color_blind = true;
//                if (intake_status == IntakeStatus.HOLDING_VERTICAL) {
//                    upper.toGroundJunction();
//                    upper.setIntakeMovePosition(0);
//                    intake_status = IntakeStatus.HOLDING_HORIZONTAL;
//                    global_drive_power = 0.4;
//                    global_drive_turn_ratio = 0.4;
//                } else {
//                    upper.grab();
//                    intake_status = IntakeStatus.HOLDING_VERTICAL;
//                    global_drive_power = 1;
//                    global_drive_turn_ratio = 1;
//                }
//            }
//
//            if (intake_status == IntakeStatus.EMPTY_HORIZONTAL) {
//                if (color_detected.toTrue() && !color_blind) {
////                    gamepad1.rumble(200);
//                    upper.grab();
//                    global_drive_power = 1;
//                    global_drive_turn_ratio = 1;
//                    cone_pos_vec = getIntakePos();
//                    intake_status = IntakeStatus.HOLDING_VERTICAL;
//                    color_blind = true;
//                }
//                if (dpad_up.toTrue()) {
//                    color_blind = false;
//                    upper.toAim(INTAKE_ARM_POSITIONS[4]);
//                } else if (dpad_down.toTrue()) {
//                    color_blind = false;
//                    upper.toAim((INTAKE_ARM_POSITIONS[3] + INTAKE_ARM_POS_LEVEL) / 2);
//                }
//            } else {
//                if ((dpad_up.toTrue() || dpad_down.toTrue() || button_y.toTrue()) && intake_status == IntakeStatus.HOLDING_VERTICAL) {
//                    upper.trans();
//                    global_drive_turn_ratio = 1;
//                    global_drive_power = 1;
//                    intake_status = IntakeStatus.EMPTY_VERTICAL;
//                }
//                if (dpad_up.toTrue()) {
//                    upper.toMidJunction();
//                    global_drive_power = 0.4;
//                    global_drive_turn_ratio = 0.4;
//                } else if (dpad_down.toTrue()) {
//                    upper.toLowJunction();
//                    global_drive_power = 0.4;
//                    global_drive_turn_ratio = 0.4;
//                } else if (button_y.toTrue()) {
//                    upper.toHighJunction();
//                    global_drive_power = 0.4;
//                    global_drive_turn_ratio = 0.35;
//                }
//            }
//
//            if (upper_release.toTrue()) {
//                //x
//                color_blind = false;
//                switch (intake_status) {
//                    case EMPTY_HORIZONTAL:
//                        upper.setIntakeMovePosition(20);
//                        upper.toOriginal();
//                        intake_status = IntakeStatus.EMPTY_VERTICAL;
//                        break;
//                    case HOLDING_VERTICAL:
//                        upper.setIntakeMovePosition(10);
//                        upper.toGroundJunction();
//                        intake_status = IntakeStatus.HOLDING_HORIZONTAL;
//                        break;
//                    case EMPTY_VERTICAL:
//                        upper.setIntakeMovePosition(0);
//                        global_drive_power = 0.1;
//                        upper.eject_action(0.1, 90);
//                        upper.sleep_with_drive(200);
//                        gamepad1.rumble(100);
//                        upper.toOriginal();
//                        break;
//                    case HOLDING_HORIZONTAL:
//                        upper.setHand(0);
//                        global_drive_power = 0.1;
//                        upper.sleep_with_drive(200);
//                        upper.toOriginal();
//                        intake_status = IntakeStatus.EMPTY_VERTICAL;
//                        break;
//                }
//                global_drive_power = 1;
//                global_drive_turn_ratio = 1;
//            }
//        }
//    }
//
//    private void logic_period() {
//        XCYBoolean.bulkRead();
//        current_pos = drive.getPoseEstimate();
//        current_intake_length = upper.getIntakeMoveLength();
//    }
//
//    private void drive_period() {
//        double x = (-this.gamepad1.left_stick_y) * 0.5 + (-this.gamepad1.right_stick_y) * 0.5;
//        double y = (-this.gamepad1.left_stick_x) * 0.5 + (-this.gamepad1.right_stick_x) * 0.5;
//        double turn_val;
//        if ((gamepad1.left_stick_y > 0.0001 && gamepad1.right_stick_y < -0.0001)||(gamepad1.right_stick_y > 0.0001 && gamepad1.left_stick_y < -0.0001))
//            turn_val = (gamepad1.left_stick_y) - (gamepad1.right_stick_y);
//        else
//            turn_val=0;
//        Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
//        double corrected_rad = fast_stick.angle() - current_pos.getHeading();
//        while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
//        while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
//        if (Math.abs(corrected_rad) < Math.PI / 5) {
//            double div = clamp(
//                    Math.toDegrees(corrected_rad) / 20, 1)
//                    * max_turn_assist_power * fast_stick.norm();
//            turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
//        }
//        Pose2d power = (new Pose2d(x, y, turn_val * global_drive_turn_ratio)).times(global_drive_power);
//
//        drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
//        drive.update();
//    }
//
//    private int intake_pos_real = -600;
//    private int intake_pos_temp = -600;
//
//    private void intake_motion() {
//        if (dpad_up.toTrue()) {
//            intake_pos_temp += 20;
//        } else if (dpad_down.toTrue()) {
//            intake_pos_temp -= 20;
//        }
//        upper.setIntakeMoveLength(intake_pos_temp);
//    }
//
//    private Vector2d getIntakePos() {
//        return current_pos.vec().plus(new Vector2d(current_intake_length, 0).rotated(current_pos.getHeading()));
//    }
//}
