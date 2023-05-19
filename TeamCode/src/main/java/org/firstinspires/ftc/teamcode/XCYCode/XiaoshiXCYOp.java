package org.firstinspires.ftc.teamcode.XCYCode;


import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;
import static org.firstinspires.ftc.teamcode.XCYCode.XCYTeleOp.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public class XiaoshiXCYOp extends LinearOpMode {
   private XCYMecanumDrive drive;
   private Servo liftArmServo, intakeArmServo, doorServo;
   private ColorSensor colorSensor;
   private double global_drive_power = 1;
   private double global_drive_turn_ratio = 1;
   private IntakeStatus intake_status = IntakeStatus.EMPTY_VERTICAL;

   private Pose2d current_pos, release_pos = new Pose2d();
   private Vector2d cone_pos_vec = new Vector2d();
   private NanoClock time;

   private TeleOpSuperStructure upper;
   private XCYBoolean chassis_to_position;

   public static double backArmPos = 0.62;

   protected boolean isSideRed = false;

   @Override
   public void runOpMode() {
      PhotonCore.enable();
      time = NanoClock.system();
      drive = new XCYMecanumDrive(hardwareMap);
      colorSensor = hardwareMap.get(ColorSensor.class, "color");
      doorServo = hardwareMap.get(Servo.class, HardwareConstant.doorServo);
      intakeArmServo = hardwareMap.get(Servo.class, HardwareConstant.intakeArm);
      liftArmServo = hardwareMap.get(Servo.class, HardwareConstant.liftServo);

      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.getLocalizer().setPoseEstimate(Configurations.get_pos_from_csv());
      XCYBoolean reset_imu = new XCYBoolean(() -> (gamepad1.back && gamepad1.a) || (gamepad2.back && gamepad2.x));
      XCYBoolean toLowJunction = new XCYBoolean(() -> !gamepad1.back && gamepad1.a);
      XCYBoolean release_upper = new XCYBoolean(() -> gamepad1.back);
      XCYBoolean reset_lifter = new XCYBoolean(() -> gamepad2.back && gamepad2.a);
      XCYBoolean reset_intake = new XCYBoolean(() -> gamepad2.back && gamepad2.b);
      XCYBoolean internal_cone_save = new XCYBoolean(() -> gamepad2.back && gamepad2.y);
      chassis_to_position = new XCYBoolean(() -> gamepad1.start);

      XCYBoolean manual_aim_mode = new XCYBoolean(() -> gamepad2.a && !gamepad2.back);
      XCYBoolean intake_action = new XCYBoolean(() -> gamepad2.b && !gamepad2.back);
      XCYBoolean release = new XCYBoolean(() -> gamepad2.x && !gamepad2.back);
      XCYBoolean button_y = new XCYBoolean(() -> gamepad2.y && !gamepad2.back);
      XCYBoolean newDoor = new XCYBoolean(() -> gamepad2.back && gamepad2.start);
      XCYBoolean dpad_down = new XCYBoolean(() -> gamepad2.dpad_down);
      XCYBoolean dpad_up = new XCYBoolean(() -> gamepad2.dpad_up);
      XCYBoolean auto_intake = new XCYBoolean(() -> gamepad2.left_bumper && intake_status != IntakeStatus.HOLDING_HORIZONTAL);
      XCYBoolean intake_max = new XCYBoolean(() -> gamepad2.right_bumper);
      upper = new TeleOpSuperStructure(hardwareMap,
              this::opModeIsActive,
              () -> {
                 logic_period();
                 drive_period();
              });
      upper.setSideIsRed(isSideRed);
      XCYBoolean color_detected = new XCYBoolean(() -> upper.isConeDetected());

      waitForStart();
      int lift_location = 0;
      boolean color_blind = true;
      int intakeAimPosIndex = 0;
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
            XCYBoolean aim_ready = new XCYBoolean(() -> Math.abs(AngleUnit.normalizeRadians(current_pos.getHeading() - target_angle)) < Math.toRadians(20));
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

         if (release_upper.toFalse()) {
            upper.setJunctionAimServo(0.25);
            doorServo.setPosition(DOOR_RELEASE);
            upper.setSpinPosition(SPIN_MID);
            liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
            upper.setLifter(LIFT_POS_MIN, 1);
            intake_status = IntakeStatus.EMPTY_VERTICAL;
         }

         if (newDoor.get()) {
            doorServo.setPosition(0.73);
         }

         if (reset_lifter.toFalse()) {
            upper.runtimeResetLifter();
         } else if (reset_intake.toTrue()) {
            upper.runtimeResetIntake();
         } else if (internal_cone_save.toTrue()) {
            upper.internalConeSaveDeep();
         } else if (reset_imu.toFalse()) {
            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
         }

         if (gamepad1.x) {
            if (lift_location == 1) doorServo.setPosition(0.05);
            else doorServo.setPosition(0.05);
            upper.setJunctionAimServo(0.25);
            release_pos = current_pos;
         } else {
            if (toLowJunction.toTrue()) {
               upper.toLowJunction();
               upper.setJunctionAimServo(backArmPos);
               lift_location = 1;
            } else if (gamepad1.b) {
               upper.toMidJunction();
               upper.setJunctionAimServo(0.1);
               lift_location = 2;
            } else if (gamepad1.y) {
               intake_status = IntakeStatus.HOLDING_HORIZONTAL;
               upper.toHighJunction();
               upper.setJunctionAimServo(0.1);
               lift_location = 3;
            }
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
            if (intake_status == IntakeStatus.HOLDING_VERTICAL) {
               upper.toGroundJunction();
               intake_status = IntakeStatus.HOLDING_HORIZONTAL;
               global_drive_power = 0.6;
               global_drive_turn_ratio = 0.3;
            } else {
               intake_status = IntakeStatus.HOLDING_VERTICAL;
               if (intakeAimPosIndex>0)
                  upper.grab(120, 0);
               else
                  upper.grab(90, 55);
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
            }
            color_blind = true;
         }

         if (gamepad2.start&&!gamepad2.back) color_blind = false;

         if (button_y.toTrue()) {
            if (intake_status==IntakeStatus.EMPTY_VERTICAL) {
               upper.trans();
               intake_status = IntakeStatus.EMPTY_VERTICAL;
            } else {
               upper.handSave();
            }
         }

         if (intake_status == IntakeStatus.EMPTY_HORIZONTAL) {
            if (dpad_up.toTrue()) {
               color_blind = true;
               intakeAimPosIndex = Range.clip(intakeAimPosIndex + 1, 0, 4);
               upper.toAim(INTAKE_ARM_POSITIONS[intakeAimPosIndex]);
               upper.setTransLiftPos(TRANS_LIFT_POS_ADD[intakeAimPosIndex]);
            } else if (dpad_down.toTrue()) {
               color_blind = true;
               intakeAimPosIndex = Range.clip(intakeAimPosIndex - 1, 0, 4);
               upper.toAim(INTAKE_ARM_POSITIONS[intakeAimPosIndex]);
               upper.setTransLiftPos(TRANS_LIFT_POS_ADD[intakeAimPosIndex]);
            }
            if (color_detected.toTrue() && !color_blind) {
               cone_pos_vec = getIntakePos();
               color_blind = true;
               intake_status = IntakeStatus.HOLDING_VERTICAL;
               global_drive_power = 0.2;
               if (intakeAimPosIndex>0)
                  upper.grab(120, 0);
               else
                  upper.grab(90, 55);
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
            }
         } else if (intake_status == IntakeStatus.HOLDING_VERTICAL) {
            if (dpad_up.isChanged()) {
               upper.transHigh();
               global_drive_turn_ratio = 1;
               global_drive_power = 1;
               intake_status = IntakeStatus.EMPTY_VERTICAL;
            } else if (dpad_down.isChanged()) {
               upper.trans();
               global_drive_turn_ratio = 1;
               global_drive_power = 1;
               intake_status = IntakeStatus.EMPTY_VERTICAL;
            }
         } else if (intake_status == IntakeStatus.HOLDING_HORIZONTAL) {
            double spin_base_power = (gamepad2.dpad_right ? -0.4 : 0) + (gamepad2.dpad_left ? 0.4 : 0);
            upper.setSpinPower(spin_base_power);
         }
//         else if (intake_status == IntakeStatus.EMPTY_VERTICAL) {
//            if (dpad_up.toTrue()) {
//               upper.transHigh();
//            } else if (dpad_down.toTrue()) {
//               upper.trans();
//            }
//         }
         if (intake_max.toTrue()) {
            upper.setIntakeMoveLength(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE + 150, 700);
            global_drive_turn_ratio = 0.3;
         }
         if (release.toFalse()) {
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
                  global_drive_power = 0.7;
                  global_drive_turn_ratio = 0.3;
                  upper.setIntakeArm(INTAKE_ARM_POS_LEVEL);
                  upper.sleep_with_drive(300);
                  upper.setHand(0);
                  upper.sleep_with_drive(300);
                  upper.setHand(INTAKE_HAND_GRIP);
                  upper.setIntakeArm(INTAKE_ARM_POS_VERTICAL);
                  intake_status = IntakeStatus.HOLDING_HORIZONTAL;
                  break;
               case EMPTY_VERTICAL:
                  global_drive_power = 1;
                  upper.toOriginal();
                  upper.runtimeResetLifter();
                  upper.runtimeResetIntake();
                  upper.setIntakeMovePosition(0);
                  global_drive_turn_ratio = 1;
                  break;
               case HOLDING_HORIZONTAL:
                  upper.setHand(0);
                  upper.setIntakeArm(upper.getIntakeArmPos() + 0.07); //TODO
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

   private void drive_period3() {
      double x = -gamepad2.left_stick_y;
      double y = -gamepad2.left_stick_x;
      drive.setSimpleMovePosition(new Pose2d(drive.getSimpleMovePosition().vec().plus(new Vector2d(
              x * period_time_sec * 150,
              y * period_time_sec * 150
      )),
              cone_pos_vec.minus(current_pos.vec()).angle()));
      drive.simpleMovePeriod(1);
   }

   void drive_period() {
      if (chassis_to_position.toTrue()) {
         drive.initSimpleMove(release_pos);
         while (chassis_to_position.get()) {
            logic_period();
            drive.simpleMovePeriod(1);
         }
      } else {
         current_pos = drive.getPoseEstimate();
         Pose2d SlowPower = (new Pose2d((
                 gamepad1.dpad_up ? 0.3 : 0) + (gamepad1.dpad_down ? -0.3 : 0),
                 (gamepad1.dpad_right ? -0.3 : 0) + (gamepad1.dpad_left ? 0.3 : 0),
                 (gamepad1.right_bumper ? -0.2 : 0) + (gamepad1.left_bumper ? 0.2 : 0)));
         double turn_val = (this.gamepad1.left_trigger - this.gamepad1.right_trigger) * 0.9 + (gamepad2.left_trigger - gamepad2.right_trigger) * global_drive_turn_ratio * global_drive_power;

         Vector2d vec = new Vector2d(
                 (-gamepad1.left_stick_y) * 0.2 + (-gamepad1.right_stick_y) * 0.7 + (-gamepad2.left_stick_y * 0.4 + -gamepad2.right_stick_y * 0.6) * global_drive_power,
                 (-gamepad1.left_stick_x) * 0.2 + (-gamepad1.right_stick_x) * 0.7 + (-gamepad2.left_stick_x * 0.4 + -gamepad2.right_stick_x * 0.6) * global_drive_power)
                 .rotated(-drive.getPoseEstimate().getHeading());

         if (vec.norm() > 0.0001) {
            vec = new Vector2d(
                    vec.getX() + Math.copySign(0.06, vec.getX()),
                    vec.getY() + Math.copySign(0.06, vec.getY())
            );
         }

         Vector2d fast_stick = new Vector2d(-gamepad2.right_stick_y, -gamepad2.right_stick_x);
         double corrected_rad = fast_stick.angle() - current_pos.getHeading();
         while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
         while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
         if (Math.abs(corrected_rad) < Math.PI / 5) {
            double div = clamp(
                    Math.toDegrees(corrected_rad) / 20, 1)
                    * 0.4 * fast_stick.norm();
            turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
         }

         drive.setWeightedDrivePower(new Pose2d(vec, turn_val).plus(SlowPower));
         drive.update();
      }
   }


   private double last_time_sec;
   private double period_time_sec;
   double current_intake_length;

   private void logic_period() {
      XCYBoolean.bulkRead();
      current_pos = drive.getPoseEstimate();
      current_intake_length = upper.getIntakeMoveLength();
//      telemetry.addData("distance", upper.getIntakeMoveLength());
//            telemetry.addData("offset", upper.getJunctionOffset());
//      telemetry.addData("current", upper.getServoCurrent());
      period_time_sec = time.seconds() - last_time_sec;
      telemetry.addData("elapse time", period_time_sec * 1000);
      last_time_sec = time.seconds();
//      telemetry.addData("is busy", upper.isBusy());
      telemetry.addData("color blue", colorSensor.blue());
      telemetry.addData("color red", colorSensor.red());
      telemetry.update();
   }

   private Vector2d getIntakePos() {
      return current_pos.vec().plus(new Vector2d(current_intake_length, 0).rotated(current_pos.getHeading()));
   }
}
