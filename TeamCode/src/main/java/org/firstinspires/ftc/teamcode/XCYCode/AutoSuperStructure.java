package org.firstinspires.ftc.teamcode.XCYCode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

public class AutoSuperStructure extends XCYSuperStructure {

   private BooleanSupplier global_time;

   public AutoSuperStructure(HardwareMap hardwareMap, BooleanSupplier continueCondition, Runnable drivePeriod) {
      super(hardwareMap, continueCondition, drivePeriod);
   }

   public void setGlobalTime(BooleanSupplier global_time) {
      this.global_time = global_time;
   }

   public static final int intakeSlowRange = 150;

   public void expand(double servo_pos, double expand_pos_mm) throws Exception {
      if (!continue_condition.getAsBoolean()) throw new InterruptedException();
      intakeArmServo.setPosition(servo_pos);
      setHand(0);
      long start_time = System.currentTimeMillis();

      while (continue_condition.getAsBoolean() && getIntakeMoveLength() - (expand_pos_mm - intakeSlowRange) > -35) {
         drive_period.run();
         setIntakeMoveLength(Math.max(MIN_INTAKE_MOVE_DISTANCE, expand_pos_mm - intakeSlowRange));
//                  if (System.currentTimeMillis() - start_time > 5000) throw new StructureJamException(); //TODO
         if (System.currentTimeMillis() - start_time > 1700) throw new TimeoutException();
         if (global_time.getAsBoolean()) throw new GlobalTimeoutException();
      }
      start_time = System.currentTimeMillis();
      while (continue_condition.getAsBoolean() && !detection_condition.getAsBoolean()) {
         drive_period.run();
         setIntakeMoveLength(Math.max(MIN_INTAKE_MOVE_DISTANCE, expand_pos_mm), 500);
         if (System.currentTimeMillis() - start_time > 2000) throw new TimeoutException();
         if (global_time.getAsBoolean()) throw new GlobalTimeoutException();
      }
   }

   public void trans() throws Exception {
      if (!continue_condition.getAsBoolean()) throw new InterruptedException();
      if (isConeHigh()) {
         if (internal_trans_high(true)) throw new StructureJamException();
      } else {
         if (internal_trans(true)) throw new StructureJamException();
      }
   }

   @Override
   public void setSideIsRed(boolean isRed) {
      super.setSideIsRed(isRed);
      if (isRed) {
         RED_BY_BLUE = 0.28;
         BLUE_BY_GREEN = 0.98;
         RED_BY_GREEN = 0.27;
      } else {
         RED_BY_BLUE = 0.37;
         BLUE_BY_GREEN = 0.67;
         RED_BY_GREEN = 0.58;
      }
   }

   public void transHigh() throws Exception {
      if (internal_trans_high(true)) throw new StructureJamException();
   }

   public void transLow() throws Exception {
      if (internal_trans(true)) throw new StructureJamException();
   }

   public void eject_ready(int lift_target_pos, boolean expand, double intakeArmPos) {
      if (!continue_condition.getAsBoolean()) return;
      drive_period.run();
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
      setHand(0);
      int liftMotorPos = liftMotor.getCurrentPosition();
      setLifter(lift_target_pos, 1);
      setSpinPosition(spin_pos);
      while (Math.abs(liftMotorPos - lift_target_pos) > lift_tolerance && continue_condition.getAsBoolean()) {
         liftMotorPos = liftMotor.getCurrentPosition();
         if (liftMotorPos > lift_target_pos * 0.2) {
            doorServo.setPosition(DOOR_HOLD);
         }

         if (liftMotorPos > lift_target_pos * 0.3) {
            liftArmServo.setPosition(LIFT_ARM_POS_EJECT_MIN);
         }

         if (Math.abs(spinBase.getCurrentPosition()) > SPIN_AMPLITUDE * 0.7)
            junctionAimServoOut();
         drive_period.run();
      }
      junctionAimServoOut();
      sleep_with_drive(200);
   }

   private double RED_BY_BLUE;
   private double BLUE_BY_GREEN;
   private double RED_BY_GREEN;

   public boolean checkConeValid() {
      double r = colorLow.red();
      double b = colorLow.blue();
      double g = colorLow.green();
      double CONFIDENCE_RANGE = 0.21;
      return Math.abs(RED_BY_BLUE - Math.min(r / b, b / r)) < CONFIDENCE_RANGE &&
              Math.abs(BLUE_BY_GREEN - Math.min(b / g, g / b)) < CONFIDENCE_RANGE &&
              Math.abs(RED_BY_GREEN - Math.min(r / g, g / r)) < CONFIDENCE_RANGE;
   }

   public String getConeData() {
      double r = colorLow.red();
      double b = colorLow.blue();
      double g = colorLow.green();
      return "RED_BY_BLUE: " + Math.min(r / b, b / r) +
              "\nBLUE_BY_GREEN: " + Math.min(b / g, g / b) +
              "\nRED_BY_GREEN: " + Math.min(r / g, g / r) +
              "\nis right cone: " + checkConeValid();
   }

   //TODO
   public boolean checkColorSensor() {
      final int CYCLES = 50;
      int prev_val = colorLow.alpha();
      int new_val;
      for (int i = 0; i < CYCLES; i++) {
         new_val = colorLow.alpha();
         if (prev_val != new_val) return true;
         drive_period.run();
         sleep_with_drive(10);
      }
      return false;
   }
}
