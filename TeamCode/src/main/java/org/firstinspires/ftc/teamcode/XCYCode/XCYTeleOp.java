package org.firstinspires.ftc.teamcode.XCYCode;


import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.i2cdrivers.QwiicLEDStick;

import java.util.List;

@Config
public class XCYTeleOp extends LinearOpMode {

   private static final double max_turn_assist_power = 0.4;
   private NanoClock time;

   private double global_drive_power = 1;
   private double global_drive_turn_ratio = 1;
   private TeleOpSuperStructure upper;

   enum IntakeStatus {
      EMPTY_HORIZONTAL, HOLDING_VERTICAL, HOLDING_HORIZONTAL, EMPTY_VERTICAL
   }

   protected boolean isRedSide;
   protected boolean noGroundMode;

   public static final double x_static_compensation = 0.06;
   public static final double y_static_compensation = 0.06;

   private IntakeStatus intake_status = IntakeStatus.EMPTY_VERTICAL;
   private XCYMecanumDrive drive;
   private Pose2d current_pos, lastEjectPos;
   private Vector2d cone_pos_vec = new Vector2d(50, -1200);
   double current_intake_length;
   private XCYBoolean dpad_down, to_last_eject;
   private XCYBoolean dpad_up;
   private List<LynxModule> allHubs;

   /**
    * left_stick/trigger: 移动 =<p>
    * <p>
    * a: 打开瞄准模式<p>
    * b: 夹取舵机垂直 -- 降低舵机 -- 松手<p>
    * x: 释放/回收<p>
    * y: 高位<p>
    * dpad up: 中位<p>
    * dpad down: 低位<p>
    * bumper: 自动移动<p>
    * dpad up: 向前<p>
    * dpad down: 向后<p>
    */

   @Override
   public void runOpMode() throws InterruptedException {
      time = NanoClock.system();
//      PhotonCore.enable();//TODO
      PhotonCore.experimental.setSinglethreadedOptimized(true);
      allHubs = hardwareMap.getAll(LynxModule.class);
      upper = new TeleOpSuperStructure(
              hardwareMap,
              this::opModeIsActive,
              () -> {
                 logic_period();
                 drive_period();
              });
      upper.setSideIsRed(isRedSide);
      drive = new XCYMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.getLocalizer().setPoseEstimate(Configurations.get_pos_from_csv());
//        drive.getLocalizer().setPoseEstimate(new Pose2d());
      current_intake_length = 420;

      XCYBoolean color_detected = new XCYBoolean(() -> upper.isConeDetected());
      XCYBoolean manual_aim_mode = new XCYBoolean(() -> (gamepad1.a || gamepad2.a) && !gamepad1.back);
      XCYBoolean intake_action = new XCYBoolean(() -> (gamepad1.b || gamepad2.b) && !gamepad1.back);
      XCYBoolean upper_release = new XCYBoolean(() -> (gamepad1.x || gamepad2.x) && !gamepad1.back);
      XCYBoolean button_y = new XCYBoolean(() -> (gamepad1.y || gamepad2.y) && !gamepad1.back);
      XCYBoolean reset_lifter = new XCYBoolean(() -> gamepad1.back && gamepad1.a);
      XCYBoolean reset_intake = new XCYBoolean(() -> gamepad1.back && gamepad1.b);
      XCYBoolean reset_imu = new XCYBoolean(() -> gamepad1.back && gamepad1.x);
      XCYBoolean intake_max = new XCYBoolean(() -> gamepad1.right_bumper && intake_status == IntakeStatus.EMPTY_HORIZONTAL);
      XCYBoolean cone_stuck_deep = new XCYBoolean(() -> gamepad1.back && gamepad1.dpad_up);
      XCYBoolean cone_stuck_near = new XCYBoolean(() -> gamepad1.back && gamepad1.dpad_down);
      XCYBoolean re_trans = new XCYBoolean(() -> gamepad1.back && gamepad1.y);

      to_last_eject = new XCYBoolean(() -> gamepad1.right_bumper && intake_status == IntakeStatus.EMPTY_VERTICAL);

      dpad_down = new XCYBoolean(() -> !gamepad1.back && gamepad1.dpad_down);
      dpad_up = new XCYBoolean(() -> !gamepad1.back && gamepad1.dpad_up);
      XCYBoolean auto_intake = new XCYBoolean(() -> gamepad1.left_bumper && intake_status != IntakeStatus.HOLDING_HORIZONTAL);

      upper.set_eject_spin_pos(0);
      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        upper.setAimReleaseCondition(dpad_down, dpad_up, upper_release, button_y);
//        upper.setWebcamConfig();
      for (LynxModule module : allHubs) {
         module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }

      waitForStart();

      logic_period();

      boolean color_blind = true;
      int lift_location = 0;
      int intakeAimPosIndex = 0;
      lastEjectPos = current_pos;

      while (opModeIsActive()) {

         if (auto_intake.toTrue() && ((intake_status == IntakeStatus.EMPTY_HORIZONTAL && !upper.isConeDetected()) || intake_status == IntakeStatus.EMPTY_VERTICAL)) {
            //自动夹取
            final double intake_move_slow_range = 50;
            color_blind = false;
            double pos_err = current_pos.vec().minus(cone_pos_vec).norm();
            double intake_final_distance = Math.min(XCYSuperStructure.MAX_INTAKE_MOVE_DISTANCE, pos_err);
            double target_angle = cone_pos_vec.minus(current_pos.vec()).angle();
            Vector2d intake_vec = new Vector2d(intake_final_distance, 0).rotated(target_angle);
            drive.initSimpleMove(new Pose2d(cone_pos_vec.minus(intake_vec), target_angle));
            XCYBoolean aim_ready = new XCYBoolean(() -> Math.abs(AngleUnit.normalizeRadians(current_pos.getHeading() - target_angle)) < Math.toRadians(13));
            logic_period();
            while (!aim_ready.get() && auto_intake.get()) {
               logic_period();
               drive_period3();
            }
            if (intakeAimPosIndex == 0) {
               upper.toAim();
            }
            long time = System.currentTimeMillis();
            while ((System.currentTimeMillis() - time < 200
                    || upper.getIntakeMoveLength() < (intake_final_distance - intake_move_slow_range) - 70
                    || current_pos.minus(drive.getSimpleMovePosition()).vec().norm() > 40)
                    && auto_intake.get()) {
               upper.setIntakeMoveLength(Math.max(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE, intake_final_distance - intake_move_slow_range));
               logic_period();
               drive_period3();
            }
            time = System.currentTimeMillis();
            while (auto_intake.get() && System.currentTimeMillis() - time < 90 && !color_detected.get()) {
               upper.setIntakeMoveLength(Math.max(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE, intake_final_distance - intake_move_slow_range));
               logic_period();
               drive_period3();
            }
            while (auto_intake.get() && !color_detected.get()) {
               upper.setIntakeMoveLength(intake_final_distance + 20, 700);
               logic_period();
               drive_period3();
            }
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.3;
            upper.setIntakeMoveLength(upper.getIntakeMoveLength());
            intake_status = IntakeStatus.EMPTY_HORIZONTAL;
            aim_ready.deactivate();
         } else {
            logic_period();
            drive_period();
         }

         if (reset_lifter.toFalse()) {
            upper.runtimeResetLifter();
         } else if (reset_intake.toFalse()) {
            upper.runtimeResetIntake();
         } else if (reset_imu.toFalse()) {
            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
         } else if (cone_stuck_deep.toTrue()) {
            upper.internalConeSaveDeep();
         } else if (cone_stuck_near.toTrue()) {
            upper.internalConeSaveNear();
         } else if (re_trans.toTrue()) {
            upper.trans();
         }

         if (manual_aim_mode.toTrue()) {
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.3;
            upper.toAim();
            intakeAimPosIndex = 0;
            upper.setTransLiftPos(LIFT_POS_MIN);
            upper.sleep_with_drive(300);
            intake_status = IntakeStatus.EMPTY_HORIZONTAL;
            color_blind = false;
         }

         if (intake_action.toTrue()) {
            //b
            color_blind = true;
            if (intake_status == IntakeStatus.HOLDING_VERTICAL) {
               upper.toGroundJunction();
               intake_status = IntakeStatus.HOLDING_HORIZONTAL;
               global_drive_power = 0.6;
               global_drive_turn_ratio = 0.3;
            } else {
               intake_status = IntakeStatus.HOLDING_VERTICAL;
               if (intakeAimPosIndex > 0)
                  upper.grab(120, 0);
               else
                  upper.grab(90, 20);
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
            }
         }

         if (intake_max.toTrue()) {
            upper.setIntakeMoveLength(700, 1300);
            global_drive_turn_ratio = 0.3;
         }
         if (intake_status == IntakeStatus.EMPTY_HORIZONTAL) {
            if (dpad_up.toTrue()) {
               color_blind = false;
               intakeAimPosIndex = Range.clip(intakeAimPosIndex + 1, 0, 4);
               upper.toAim(INTAKE_ARM_POSITIONS[intakeAimPosIndex]);
               upper.setTransLiftPos(TRANS_LIFT_POS_ADD[intakeAimPosIndex]);
            } else if (dpad_down.toTrue()) {
               color_blind = false;
               intakeAimPosIndex = Range.clip(intakeAimPosIndex - 1, 0, 4);
               upper.toAim(INTAKE_ARM_POSITIONS[intakeAimPosIndex]);
               upper.setTransLiftPos(TRANS_LIFT_POS_ADD[intakeAimPosIndex]);
            }
            if (color_detected.toTrue() && !color_blind) {
               cone_pos_vec = getIntakePos();
               color_blind = true;
               intake_status = IntakeStatus.HOLDING_VERTICAL;
               global_drive_power = 0.2;
               if (intakeAimPosIndex > 0)
                  upper.grab(120, 0);
               else
                  upper.grab(90, 20);
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
               if (noGroundMode) {
                  upper.trans();
                  intake_status = IntakeStatus.EMPTY_VERTICAL;
               }
            }
         } else {
            if (dpad_up.toTrue() || dpad_down.toTrue() || button_y.toTrue()) {
               if (intake_status == IntakeStatus.HOLDING_VERTICAL) {
                  if (dpad_up.toTrue()) {
                     upper.transHigh();
                     intake_status = IntakeStatus.EMPTY_VERTICAL;
                  } else if (dpad_down.toTrue()) {
                     upper.trans();
                     intake_status = IntakeStatus.EMPTY_VERTICAL;
                  } else if (button_y.toTrue()) {
                     upper.handSave();
                  }
                  global_drive_turn_ratio = 1;
                  global_drive_power = 1;
               } else {
                  if (dpad_up.toTrue() || dpad_down.toTrue() || button_y.toTrue()) {
                     if (dpad_up.toTrue()) {
                        global_drive_power = 1;
                        global_drive_turn_ratio = 0.4;
                        lift_location = 2;
                        upper.toMidJunction();
                     } else if (dpad_down.toTrue()) {
                        global_drive_power = 1;
                        global_drive_turn_ratio = 0.4;
                        upper.toLowJunction();
                        lift_location = 1;
                     } else if (button_y.toTrue()) {
                        global_drive_power = 0.9;
                        global_drive_turn_ratio = 0.35;
                        upper.toHighJunction();
                        lift_location = 3;
                     }
                     upper.junctionAimServoBack();
                  }
               }
            } else if ((dpad_up.toFalse() || dpad_down.toFalse() || button_y.toFalse())&&intake_status != IntakeStatus.HOLDING_VERTICAL) {
               if (lift_location == 1) {
                  upper.setJunctionAimServo(0.62);
               } else {
                  upper.junctionAimServoOut();
               }
            }
         }

         if (upper_release.toFalse()) {
            //x
            color_blind = false;
            intakeAimPosIndex = 0;
            switch (intake_status) {
               case EMPTY_HORIZONTAL:
                  upper.setIntakeMovePosition(0);
                  upper.toOriginal();
                  intake_status = IntakeStatus.EMPTY_VERTICAL;
                  global_drive_power = 1;
                  global_drive_turn_ratio = 1;
                  break;
               case HOLDING_VERTICAL:
                  upper.setIntakeArm(INTAKE_ARM_POS_LOW_JUNCTION);
                  intake_status = IntakeStatus.HOLDING_HORIZONTAL;
                  global_drive_power = 0.7;
                  global_drive_turn_ratio = 0.3;
                  break;
               case EMPTY_VERTICAL:
                  lastEjectPos = current_pos;
                  upper.setIntakeMovePosition(0);
                  global_drive_power = 0.4;
                  upper.junctionAimServoBack();
                  if (lift_location == 1)
                     upper.eject_action(0, 300, 0);
                  else
                     upper.eject_action(0.03, 200, 0);
                  global_drive_power = 1;
                  if (lift_location != 1)
                     upper.post_eject();
                  upper.toOriginal();
                  upper.runtimeResetLifter();
                  upper.runtimeResetIntake();
                  upper.setIntakeMovePosition(0);
                  global_drive_turn_ratio = 1;
                  break;
               case HOLDING_HORIZONTAL:
                  upper.setHand(0);
                  upper.setIntakeArm(upper.getIntakeArmPos() + 0.05); //TODO
                  global_drive_power = 0.3;
                  upper.sleep_with_drive(100);
                  upper.toOriginal();
                  intake_status = IntakeStatus.EMPTY_VERTICAL;
                  global_drive_power = 1;
                  global_drive_turn_ratio = 1;
                  break;
            }
         }
      }
   }

   private double last_time_sec;
   private double period_time_sec;

   private void logic_period() {
      XCYBoolean.bulkRead();
      current_pos = drive.getPoseEstimate();
      current_intake_length = upper.getIntakeMoveLength();
      telemetry.addData("distance", upper.getIntakeMoveLength());
//            telemetry.addData("offset", upper.getJunctionOffset());
      telemetry.addData("current", upper.getServoCurrent());
      period_time_sec = time.seconds() - last_time_sec;
      telemetry.addData("elapse time", period_time_sec * 1000);
      last_time_sec = time.seconds();
      telemetry.addData("is busy", upper.isBusy());
      if (intake_status == IntakeStatus.HOLDING_VERTICAL)
         telemetry.addData("is cone high", upper.isConeHigh());
      telemetry.update();
      for (LynxModule module : allHubs) {
         module.clearBulkCache();
      }
   }

   private void drive_period() {
      if (to_last_eject.toTrue()) {
         drive.initSimpleMove(lastEjectPos);
      } else if (to_last_eject.get()) {
         drive_period2();
      } else {
         double x = -gamepad1.left_stick_y * 0.4 + -gamepad1.right_stick_y * 0.6;
         double y = -gamepad1.left_stick_x * 0.4 + -gamepad1.right_stick_x * 0.6;
         double turn_val = (this.gamepad1.left_trigger - this.gamepad1.right_trigger);
         Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
         double corrected_rad = fast_stick.angle() - current_pos.getHeading();
         while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
         while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
         if (Math.abs(corrected_rad) < Math.PI / 5) {
            double div = clamp(
                    Math.toDegrees(corrected_rad) / 20, 1)
                    * max_turn_assist_power * fast_stick.norm();
            turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
         }
         Pose2d power = (new Pose2d(x, y, turn_val * global_drive_turn_ratio)).times(global_drive_power);

         if ((Math.abs(power.getHeading()) > 0.65) && intake_status == IntakeStatus.EMPTY_HORIZONTAL) {
            upper.toOriginal();
            upper.setIntakeMovePosition(20);
            intake_status = IntakeStatus.EMPTY_VERTICAL;
         }
         drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
         drive.update();
      }
   }

   private void drive_period2() {
      double x = -gamepad1.left_stick_y;
      double y = -gamepad1.left_stick_x;
      double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
      drive.setSimpleMovePosition(drive.getSimpleMovePosition().plus(new Pose2d(
              x * period_time_sec * 150,
              y * period_time_sec * 150,
              turn_val * period_time_sec * Math.toRadians(30)
      )));
      drive.simpleMovePeriod(1);
   }

   private void drive_period3() {
      double x = -gamepad1.left_stick_y;
      double y = -gamepad1.left_stick_x;
      drive.setSimpleMovePosition(new Pose2d(drive.getSimpleMovePosition().vec().plus(new Vector2d(
              x * period_time_sec * 150,
              y * period_time_sec * 150
      )),
              cone_pos_vec.minus(current_pos.vec()).angle()));
      drive.simpleMovePeriod(1);
   }

   private Vector2d getIntakePos() {
      return current_pos.vec().plus(new Vector2d(current_intake_length, 0).rotated(current_pos.getHeading()));
   }
}
