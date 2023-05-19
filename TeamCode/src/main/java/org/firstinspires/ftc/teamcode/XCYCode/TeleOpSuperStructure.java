package org.firstinspires.ftc.teamcode.XCYCode;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.BooleanSupplier;

@Config
public class TeleOpSuperStructure extends XCYSuperStructure {
   private final boolean DEBUG = true;

   public static final int WIDTH = 640;
   public static final int HEIGHT = 480;
   public static int WHITE_BALANCE = 5200;
   private final NanoClock time;

   public TeleOpSuperStructure(HardwareMap hardwareMap, BooleanSupplier continueCondition, Runnable drivePeriod) {
      super(hardwareMap, continueCondition, drivePeriod);
      time = NanoClock.system();
   }

   public void toOriginal() {
      doorServo.setPosition(DOOR_RELEASE);
      setLifter(LIFT_POS_MIN, 1);
      setSpinPosition(SPIN_MID);
      while (Math.abs(spinBase.getCurrentPosition() - SPIN_MID) > 40 || Math.abs(liftMotor.getCurrentPosition() - LIFT_POS_MIN) > 15) {
         setSpinPosition(SPIN_MID);
         setLifter(LIFT_POS_MIN, 1);
         drive_period.run();
      }
      liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
   }

   private double last_time;

   public void setServoVel(double hand_vel, double arm_vel) {
      double cur_time = time.seconds();
      double dt = cur_time - last_time;
      if (handServo0.getPosition() < INTAKE_HAND_0_POS - 0.001) {
         double hand_servo_pos = handServo1.getPosition();
         handServo1.setPosition(Range.clip(hand_servo_pos + dt * hand_vel, INTAKE_HAND_1_POS + (INTAKE_HAND_GRIP - 0.15), INTAKE_HAND_1_POS));
      } else {
         double hand_servo_pos = handServo0.getPosition();
         handServo0.setPosition(Range.clip(hand_servo_pos + dt * hand_vel, INTAKE_HAND_0_POS, INTAKE_HAND_0_POS - (INTAKE_HAND_GRIP - 0.15)));
      }
      double intake_servo_pos = intakeArmServo.getPosition();
      intakeArmServo.setPosition(Range.clip(intake_servo_pos - dt * arm_vel, INTAKE_ARM_POS_VERTICAL, INTAKE_ARM_POS_LEVEL));
      last_time = cur_time;
   }

   public void initPowerServoMode(boolean isHand1) {
      last_time = time.seconds();
      if (isHand1) {
         handServo0.setPosition(INTAKE_HAND_0_POS - 0.1);
         handServo1.setPosition(INTAKE_HAND_1_POS + INTAKE_HAND_GRIP);
      } else {
         handServo0.setPosition(INTAKE_HAND_0_POS - INTAKE_HAND_GRIP);
         handServo1.setPosition(INTAKE_HAND_1_POS + 0.1);
      }
   }

   public void auto_grab() {
      if (!continue_condition.getAsBoolean()) return;
      setHand(INTAKE_HAND_GRIP);
      sleep_with_drive(100);
      drive_period.run();
   }

   public double getServoCurrent() {
      return PhotonCore.CONTROL_HUB.getGpioBusCurrent(CurrentUnit.MILLIAMPS);
   }

   public double getIntakeArmPos() {
      return intakeArmServo.getPosition();
   }

   public void preEjectAction(double lift_arm_pos_add) {
      super.preEjectAction(lift_arm_pos_add);
      spinBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      spinBase.setPower(0);
   }

   public void trans() {
      super.internal_trans(false);
      setIntakeMovePosition(0);
   }

   public void grab() {
      grab(120);
   }

   public void grab(int time) {
      if (!continue_condition.getAsBoolean()) return;
      setHand(INTAKE_HAND_GRIP);
      setIntakeMovePow(0);
      sleep_with_drive(time);
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
      sleep_with_drive(150);
      setIntakeMovePositionPower(INTAKE_MOVE_WAY_POS, 1);
      drive_period.run();
   }

   public void handSave(){
      intakeArmServo.setPosition(intakeArmServo.getPosition()-0.02);
      sleep_with_drive(35);
      setHand(0);
      sleep_with_drive(25);
      intakeArmServo.setPosition(intakeArmServo.getPosition()+0.02);
      sleep_with_drive(35);
      setHand(INTAKE_HAND_GRIP);
   }

   public void transHigh() {
      internal_trans_high(false);
      setIntakeMovePosition(0);
   }
}
