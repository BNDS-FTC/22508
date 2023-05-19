package org.firstinspires.ftc.teamcode.XCYCode.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.XCYCode.Configurations;
import org.firstinspires.ftc.teamcode.XCYCode.TeleOpSuperStructure;
import org.firstinspires.ftc.teamcode.XCYCode.XCYBoolean;
import org.firstinspires.ftc.teamcode.XCYCode.XCYMecanumDrive;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;


@TeleOp(name = "Xiaoshi Ziming Op")
//@Disabled
public class XiaoshizimingOp extends LinearOpMode {
   private XCYMecanumDrive drive;
   private Servo liftArmServo, intakeArmServo, doorServo;
   private ColorSensor colorSensor;

   public static double intake_pow_ratio = .5;
   public static double lift_servo_pos_high = 0.75;

   public static int high_lift_pos = 600;
   public static int mid_lift_pos = 200;

   public static double beacon_liftarm_pos = 0.83;
   public static double beacon_door_pos = 0.428;

   private int lift_pos_add = 0;
   private int spin_pos_add = 0;

   private boolean right_auto;

   private Pose2d current_pos = new Pose2d(), recorded_pos = new Pose2d();

   private XCYBoolean chassis_to_pos;

   @Override
   public void runOpMode() {
      chassis_to_pos = new XCYBoolean(() -> gamepad1.start);
      Telemetry telemetry_ = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
      drive = new XCYMecanumDrive(hardwareMap);
      colorSensor = hardwareMap.get(ColorSensor.class, "color");
      doorServo = hardwareMap.get(Servo.class, HardwareConstant.doorServo);
      intakeArmServo = hardwareMap.get(Servo.class, HardwareConstant.intakeArm);
      liftArmServo = hardwareMap.get(Servo.class, HardwareConstant.liftServo);
      XCYBoolean reset_lifter = new XCYBoolean(() -> gamepad2.right_bumper);
      XCYBoolean reset_intake = new XCYBoolean(() -> gamepad2.left_bumper);
      XCYBoolean internal_cone_save = new XCYBoolean(() -> gamepad2.right_stick_button);
      XCYBoolean beacon = new XCYBoolean(() -> gamepad1.back && gamepad1.a);
      XCYBoolean junctionLow = new XCYBoolean(()->gamepad1.a);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.getLocalizer().setPoseEstimate(Configurations.get_pos_from_csv());
      XCYBoolean reset_imu = new XCYBoolean(() -> gamepad1.back && gamepad1.y);

      int intakeIndex = 0;
      TeleOpSuperStructure upper = new TeleOpSuperStructure(hardwareMap,
              this::opModeIsActive,
              this::drive_period);
      waitForStart();
      boolean lock_upper = false;
      boolean intake_lock = false, high_trans_lock = false;
      double lift_power = 1;
      long last_period_time = System.currentTimeMillis();
      int lift_location = 0;
      while (opModeIsActive()) {

         if (reset_imu.toFalse()) {
            drive.getLocalizer().setPoseEstimate(new Pose2d());
            drive.setPoseEstimate(new Pose2d());
         }
         long current_time = System.currentTimeMillis();
         drive_period();

         if (gamepad1.back&&!gamepad1.a) {
            lock_upper = false;
            doorServo.setPosition(DOOR_RELEASE);
            liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
            upper.setLifter(LIFT_POS_MIN, 0.5);
//            upper.toOriginal();
//            upper.runtimeResetLifter();
         }

         if (beacon.toTrue()) {
//            upper.liftArmServo.setPosition(0.815);
//            upper.sleep_with_drive(200);
//            upper.doorServo.setPosition(0.6);
//            upper.sleep_with_drive(200);
//            upper.liftArmServo.setPosition(beacon_liftarm_pos);
//            upper.sleep_with_drive(200);
//            upper.doorServo.setPosition(beacon_door_pos);
         }

         if (reset_lifter.toFalse()) {
            upper.runtimeResetLifter();
         } else if (reset_intake.toTrue()) {
            upper.runtimeResetIntake();
         } else if (internal_cone_save.toTrue()) {
            upper.internalConeSaveDeep();
         }
         if (gamepad1.x) {
            if (lift_location == 1) doorServo.setPosition(0.05);
            else doorServo.setPosition(0.05);
         } else {
            if (junctionLow.toTrue()) {
               lock_upper = true;
               lift_pos_add = 60;
               lift_power = 1;
               lift_location = 1;
               upper.toLowJunction();
            } else if (gamepad1.b) {
               lock_upper = true;
               lift_pos_add = mid_lift_pos;
               lift_power = 1;
               liftArmServo.setPosition(lift_servo_pos_high);
               doorServo.setPosition(DOOR_HOLD);
               lift_location = 2;
            } else if (!gamepad1.back && gamepad1.y) {
               lock_upper = true;
               lift_pos_add = high_lift_pos;
               lift_power = 1;
               liftArmServo.setPosition(lift_servo_pos_high);
               doorServo.setPosition(DOOR_HOLD);
               lift_location = 3;
            } else {
               if (!lock_upper) {
                  lift_power = 0.4;
                  lift_pos_add = LIFT_POS_MIN;
//                        if (!init_lock)
//                            liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
                  spin_pos_add = 0;
               }
//                    else {
//                        if (!last_request_lift_down && gamepad1.dpad_down) lift_pos_add -= 20;
//                        if (!last_request_lift_up && gamepad1.dpad_up) lift_pos_add += 20;
//                    }
            }
            if (lock_upper) {
               double spin_base_power = (gamepad2.right_trigger - gamepad2.left_trigger) * -0.5;
               upper.setSpinPower(spin_base_power);
            } else {
               upper.setSpinPosition(SPIN_MID);
            }
         }
         upper.setLifter(lift_pos_add, lift_power);
         if (!intake_lock) {
            double intake_move_power = -gamepad2.left_stick_y * intake_pow_ratio;
            upper.setIntakeMovePow(intake_move_power);
         }
         if (gamepad2.a) {
            intakeArmServo.setPosition(INTAKE_ARM_POS_AWAIT_TRANS);
         } else if (gamepad2.b) {
            intake_lock = false;
            lift_pos_add = 5;
            intakeArmServo.setPosition(INTAKE_ARM_POS_LEVEL);
         } else if (gamepad2.x) {
            intake_lock = true;
            upper.grab(120,30);
         } else if (gamepad2.y) {
            intake_lock = false;
            if (upper.isConeHigh() && !high_trans_lock)
               upper.transHigh();
            else
               upper.trans();
         } else if (gamepad2.dpad_down) {
            intake_lock = false;
            intakeArmServo.setPosition(INTAKE_ARM_POSITIONS[1]);
         } else if (gamepad2.dpad_up) {
            intake_lock = false;
            intakeArmServo.setPosition(INTAKE_ARM_POSITIONS[4]);
         } else if (gamepad2.dpad_right) {
            intake_lock = false;
            intakeArmServo.setPosition(INTAKE_ARM_POSITIONS[3]);
         } else if (gamepad2.dpad_left) {
            intake_lock = false;
            intakeArmServo.setPosition(INTAKE_ARM_POSITIONS[2]);
         } else if (gamepad2.back) {
            upper.setHand(0);
            sleep_with_drive(200);
            intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
         }

         if (gamepad2.start) {
            high_trans_lock = true;
         }

         last_period_time = current_time;
         telemetry.addData("spin", spin_pos_add);
         telemetry.addData("lift", lift_pos_add);
         telemetry.addData("cone high", upper.isConeHigh());
         telemetry.update();
      }
      intakeArmServo.close();
   }

   public static int intake_move_pos = -700;

   void drive_period() {
      right_auto = gamepad2.right_trigger > 0.5 || gamepad2.left_trigger > 0.5;
      XCYBoolean.bulkRead();
      current_pos = drive.getPoseEstimate();
      if (chassis_to_pos.toTrue()) {
         drive.initSimpleMove(recorded_pos);
      } else if (chassis_to_pos.get()) {
         drive.simpleMovePeriod();
      } else {
         Pose2d SlowPower = (new Pose2d((
                 gamepad1.dpad_up ? 0.3 : 0) + (gamepad1.dpad_down ? -0.3 : 0),
                 (gamepad1.dpad_right ? -0.3 : 0) + (gamepad1.dpad_left ? 0.3 : 0),
                 (gamepad1.right_bumper ? -0.2 : 0) + (gamepad1.left_bumper ? 0.2 : 0)));
         double turn_val = (this.gamepad1.left_trigger - this.gamepad1.right_trigger) * 0.9;

         Vector2d vec = new Vector2d(
                 (-gamepad1.left_stick_y) * 0.2 + (-gamepad1.right_stick_y) * 0.7,
                 (-gamepad1.left_stick_x) * 0.2 + (-gamepad1.right_stick_x) * 0.7)
                 .rotated(-drive.getPoseEstimate().getHeading());
         if (vec.norm() > 0.001) {
            vec = new Vector2d(
                    vec.getX() + Math.copySign(0.06, vec.getX()),
                    vec.getY() + Math.copySign(0.06, vec.getY())
            );
         }
         drive.setWeightedDrivePower(new Pose2d(vec, turn_val).plus(SlowPower));
         drive.update();
         if (gamepad2.x) recorded_pos = current_pos;
      }
   }

   void sleep_with_drive(double time_mm) {
      long start_time = System.currentTimeMillis();
      while ((right_auto) && opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
         drive_period();
      }
   }
}
