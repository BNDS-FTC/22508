package org.firstinspires.ftc.teamcode.XCYCode;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.function.BooleanSupplier;

@Config
public class XCYSuperStructure {
   protected final BooleanSupplier continue_condition;
   protected BooleanSupplier detection_condition;

   public void setDrivePeriod(Runnable drive_period) {
      this.drive_period = drive_period;
   }

   protected Runnable drive_period;

   public static final double MAX_INTAKE_MOVE_DISTANCE = 1000;
   public static final double MIN_INTAKE_MOVE_DISTANCE = 420;
   public static final double INTAKE_TICK_TO_MM = 1.0 / (28 * 5) * Math.PI * 32;
   public static double STATIC_COMPENSATION_DISTANCE = 0;

   protected final DcMotorEx spinBase, liftMotor, intakeMoveMotor0, intakeMoveMotor1;
   protected final Servo liftArmServo, handServo0, intakeArmServo, handServo1, doorServo, coneSaveServo;
   protected final ColorSensor colorLow, colorHigh;
   protected final Servo junctionAimServo;

   public XCYSuperStructure(HardwareMap hardwareMap,
                            BooleanSupplier continueCondition,
                            Runnable drivePeriod) {
      continue_condition = continueCondition;
      drive_period = drivePeriod;
      doorServo = hardwareMap.get(Servo.class, Configurations.HardwareConstant.doorServo);
      spinBase = hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.spin);
      liftMotor = hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.liftMotor);
      liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      liftMotor.setTargetPositionTolerance(1);
      intakeMoveMotor0 = hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.intakeMove0);
      intakeMoveMotor0.setDirection(HardwareConstant.intakeMove0Direction);
      intakeMoveMotor0.setTargetPositionTolerance(5);
      intakeMoveMotor1 = hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.intakeMove1);
      intakeMoveMotor1.setDirection(HardwareConstant.intakeMove1Direction);
      intakeMoveMotor1.setTargetPositionTolerance(5);

      intakeArmServo = hardwareMap.get(Servo.class, Configurations.HardwareConstant.intakeArm);
      handServo0 = hardwareMap.get(Servo.class, Configurations.HardwareConstant.intakeHand0);
      handServo1 = hardwareMap.get(Servo.class, Configurations.HardwareConstant.intakeHand1);
      liftArmServo = hardwareMap.get(Servo.class, Configurations.HardwareConstant.liftServo);
      coneSaveServo = hardwareMap.get(Servo.class, "servo5");
      colorLow = hardwareMap.get(ColorSensor.class, "color");
      colorHigh = hardwareMap.get(ColorSensor.class, "color2");
      junctionAimServo = hardwareMap.get(Servo.class,"backArm");
      setSideIsRed(true);
   }

   public boolean isBusy() {
      return spinBase.isBusy() || intakeMoveMotor0.isBusy();
   }

   public void setSpinPosition(int pos) {
      spinBase.setTargetPosition(pos);
      spinBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      spinBase.setPower(0.5);
   }

   public void setSpinPower(double pow) {
      spinBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      spinBase.setPower(pow);
   }

   public void setSideIsRed(boolean isRed) {
      if (isRed)
         detection_condition = () -> colorLow.red() > COLOR_DETECTION || colorHigh.red() > COLOR_DETECTION;
      else
         detection_condition = () -> colorLow.blue() > COLOR_DETECTION - 50 || colorHigh.blue() > COLOR_DETECTION;
   }

   public boolean isConeDetected() {
      return detection_condition.getAsBoolean();
   }

   public void toAim() {
      toAim(INTAKE_ARM_POS_LEVEL);
   }

   public void toAim(double intake_arm_pos) {
      setLifter(transLiftPos, 0.6);
      liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
      intakeArmServo.setPosition(intake_arm_pos);
      setHand(0);
      doorServo.setPosition(DOOR_RELEASE);
   }

   /**
    * 中杆
    */
   public static int MID_LIFT_MIN = 140;
   public static final double MID_DISTANCE_MIN = 17;

   public static final int MID_LIFT_MID = 400;
   public static final double MID_DISTANCE_MID = 25;

   public static final int MID_LIFT_MAX = 870;
   public static final double MID_DISTANCE_MAX = 32;

   /**
    * 低杆
    */
   public static int LOW_LIFT_MIN = 50;
   public static final double LOW_DISTANCE_MIN = 17;
   public static final int LOW_LIFT_MAX = 630;
   public static final double LOW_DISTANCE_MAX = 30;

   /**
    * 高杆
    */
   public static int HIGH_LIFT_MIN = 600;
   public static final double HIGH_DISTANCE_MIN = 26;
   public static final int HIGH_LIFT_MID = 800;
   public static final double HIGH_DISTANCE_MID = 33;
   public static final int HIGH_LIFT_MAX = 950;
   public static final double HIGH_DISTANCE_MAX = 42;

   public void junctionAimServoOut(){
      junctionAimServo.setPosition(1);
   }

   public void setJunctionAimServo(double pos){
      junctionAimServo.setPosition(pos);
   }

   public void junctionAimServoBack(){
      junctionAimServo.setPosition(0.3);
   }

   public void toGroundJunction() {
      setSpinPosition(SPIN_MID);
      intakeArmServo.setPosition(INTAKE_ARM_POS_GROUND);
   }
   public static double lowJunctionPos = 0.73;

   public void toLowJunction() {
      doorServo.setPosition(DOOR_HOLD);
      setSpinPosition(SPIN_MID);
      setLifter(LOW_LIFT_MIN, 0.7);
      liftArmServo.setPosition(lowJunctionPos);
   }

   public void toMidJunction() {
      doorServo.setPosition(DOOR_HOLD);
      setSpinPosition(SPIN_MID);
      setLifter(MID_LIFT_MIN + 60, 0.7);
      liftArmServo.setPosition(LIFT_ARM_POS_EJECT_MIN);
   }

   public void toHighJunction() {
      doorServo.setPosition(DOOR_HOLD);
      setSpinPosition(spin_pos);
      setLifter(HIGH_LIFT_MIN + 40, 1);
      liftArmServo.setPosition(LIFT_ARM_POS_EJECT_MIN);
   }

   public void setIntakeMovePosition(int move_pos) {
      setIntakeMovePositionPower(move_pos, 1);
   }

   public void setIntakeMovePositionPower(int pos, double pow) {
      intakeMoveMotor0.setTargetPosition(pos);
      intakeMoveMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      intakeMoveMotor0.setPower(pow);
      intakeMoveMotor1.setTargetPosition(pos);
      intakeMoveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      intakeMoveMotor1.setPower(pow);
   }

   public void setIntakeMovePositionVelocity(int pos, double vel) {
      intakeMoveMotor0.setTargetPosition(pos);
      intakeMoveMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      intakeMoveMotor0.setVelocity(vel);
      intakeMoveMotor1.setTargetPosition(pos);
      intakeMoveMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      intakeMoveMotor1.setVelocity(vel);
   }

   public double getIntakeMoveTicks() {
      return 0.5 * (intakeMoveMotor0.getCurrentPosition() + intakeMoveMotor1.getCurrentPosition());
   }

   private int getIntakeTickFromLength(double length_mm) {
      return (int) ((length_mm - MIN_INTAKE_MOVE_DISTANCE) / INTAKE_TICK_TO_MM);
   }

   public void setIntakeMoveLength(double length_mm) {
      setIntakeMovePositionPower(getIntakeTickFromLength(length_mm + STATIC_COMPENSATION_DISTANCE), 1);
   }

   public void setIntakeMoveLength(double length_mm, double vel) {
      setIntakeMovePositionVelocity(getIntakeTickFromLength(length_mm + STATIC_COMPENSATION_DISTANCE), vel);
   }

   public double getIntakeMoveLength() {
      return (getIntakeMoveTicks() * INTAKE_TICK_TO_MM) + MIN_INTAKE_MOVE_DISTANCE;
   }

   /**
    * @return true: 超时终止
    * false: 正常终止
    */
   private boolean preTrans() {
      doorServo.setPosition(DOOR_RELEASE);
      long start_time = System.currentTimeMillis();
      double intakeMovePos = getIntakeMoveTicks();
      setIntakeMovePositionPower(INTAKE_MOVE_WAY_POS, 1);
      drive_period.run();
      getIntakeMoveTicks();
      while (continue_condition.getAsBoolean() && intakeMovePos > 20) {
         intakeMovePos = getIntakeMoveTicks();
         drive_period.run();
         setIntakeMovePositionPower(INTAKE_MOVE_WAY_POS, 1);
         wait_for_trans_update();
         intakeArmServo.setPosition(
                 map(intakeMovePos,
                         400,
                         0,
                         INTAKE_ARM_POS_VERTICAL,
                         INTAKE_ARM_POS_AWAIT_TRANS));
         if (System.currentTimeMillis() - start_time > 3500) return true;
      }
      return false;
   }

   /**
    * @return true: 非正常终止
    * false: 正常终止
    */
   protected boolean internal_trans(boolean closeDoor) {
      if (preTrans()) return true;
      setIntakeMovePow(-0.15);
      intakeArmServo.setPosition(INTAKE_ARM_POS_AWAIT_TRANS);
      wait_for_trans_update();
      liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotor.setPower(-0.17);
      sleep_with_drive(60);
      intakeArmServo.setPosition(INTAKE_ARM_POS_TRANS);
      sleep_with_drive(40);
      setHand(0.);
      sleep_with_drive(40);
      setIntakeMovePow(0);
      liftArmServo.setPosition(LIFT_ARM_POS_HOLD);
      liftMotor.setPower(0);
      if (closeDoor) return false;
      sleep_with_drive(60);
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
      doorServo.setPosition(DOOR_HOLD);
      intakeMoveMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intakeMoveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      setIntakeMovePow(0);
      drive_period.run();
      setLifter(5, 1);
      drive_period.run();
      return false;
   }

   protected boolean internal_trans_high(boolean closeDoor) {
      if (preTrans()) return true;
      setIntakeMovePow(-0.15);
      intakeArmServo.setPosition(INTAKE_ARM_POS_AWAIT_TRANS);
      while (liftMotor.getCurrentPosition() < 36) {
         liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         liftMotor.setPower(0.45);
         drive_period.run();
      }
      liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotor.setPower(0);
      intakeArmServo.setPosition(INTAKE_ARM_POS_TRANS - 0.04);
      sleep_with_drive(120);
      liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotor.setPower(-0.3);
      setHand(0.);
      sleep_with_drive(100);
      liftArmServo.setPosition(LIFT_ARM_POS_HOLD);
      sleep_with_drive(40);
      liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotor.setPower(0);
      setIntakeMovePow(0);
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
      if (closeDoor) return false;
      sleep_with_drive(60);
      doorServo.setPosition(DOOR_HOLD);
      intakeMoveMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intakeMoveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      setIntakeMovePow(0);
      drive_period.run();
      setLifter(5, 1);
      drive_period.run();
      return false;
   }

   public boolean isConeHigh() {
      return colorHigh.alpha() < colorLow.alpha() / 1.8;
   }

   void wait_for_trans_update() {
      setLifter(transLiftPos, 1);
      liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
      spinBase.setTargetPosition(SPIN_MID);
      spinBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      spinBase.setPower(1);
   }

   public static final int lift_tolerance = 20;

   protected int spin_pos;

   public void setIntakeMovePos(int intake_pos) {
      this.intake_pos = intake_pos;
   }

   public void runtimeResetLifter() {
      long start_time = System.currentTimeMillis();
      while (continue_condition.getAsBoolean() && System.currentTimeMillis() - start_time < 200) {
         liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         liftMotor.setPower(-0.23);
         drive_period.run();
      }
      liftMotor.setPower(0.05);
      sleep_with_drive(100);
      liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      liftMotor.setPower(0);
   }

   public void runtimeResetIntake() {
      intakeMoveMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intakeMoveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intakeMoveMotor0.setPower(0);
      intakeMoveMotor1.setPower(0);
      drive_period.run();
      long start_time = System.currentTimeMillis();
      while (continue_condition.getAsBoolean() && System.currentTimeMillis() - start_time < 200) {
         setIntakeMovePow(-0.2);
         drive_period.run();
      }
      intakeMoveMotor0.setPower(0);
      intakeMoveMotor1.setPower(0);
      sleep_with_drive(100);
      intakeMoveMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intakeMoveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intakeMoveMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intakeMoveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intakeMoveMotor0.setPower(0);
      intakeMoveMotor1.setPower(0);
   }

   protected int intake_pos;

   public void setTransLiftPos(int transLiftPos) {
      this.transLiftPos = transLiftPos;
   }

   private int transLiftPos = LIFT_POS_MIN;

   public void set_eject_spin_pos(int pos) {
      spin_pos = pos;
   }

   public void eject_ready(int lift_target_pos, boolean expand, double intakeArmPos) {
      if (!continue_condition.getAsBoolean()) return;
      drive_period.run();
      intakeArmServo.setPosition(expand ? intakeArmPos : INTAKE_ARM_POS_AWAIT_TRANS);
      setHand(0);
      int liftMotorPos = liftMotor.getCurrentPosition();
      setSpinPosition(spin_pos);
      setLifter(lift_target_pos, 1);
      while (Math.abs(liftMotorPos - lift_target_pos) > lift_tolerance && continue_condition.getAsBoolean()) {
         liftMotorPos = liftMotor.getCurrentPosition();
         if (expand) {
            setIntakeMovePosition(intake_pos);
         }
         setLifter(lift_target_pos, 1);
         if (liftMotorPos > lift_target_pos / 3) {
            doorServo.setPosition(DOOR_HOLD);
         }
         drive_period.run();
      }
   }

   public void eject_action(double lift_arm_pos_add, int add_sleep) {
      eject_action(lift_arm_pos_add, add_sleep, 20);
   }

   public void eject_action(double lift_arm_pos_add, int add_sleep, int lift_down_val) {
      preEjectAction(lift_arm_pos_add);
      if (!continue_condition.getAsBoolean()) return;
      setLifter(Math.max(LIFT_POS_MIN, liftMotor.getCurrentPosition() - lift_down_val), 1);
      junctionAimServoBack();
      doorServo.setPosition(DOOR_RELEASE);

      long start_time = System.currentTimeMillis();
      while (continue_condition.getAsBoolean() && System.currentTimeMillis() - start_time < add_sleep) {
         drive_period.run();
      }
   }

   public void preEjectAction(double lift_arm_pos_add) {
      liftArmServo.setPosition(Range.clip(liftArmServo.getPosition() + lift_arm_pos_add, LIFT_ARM_POS_HOLD, 1));
   }

   public void setLifter(int pos, double power) {
      liftMotor.setTargetPosition(pos);
      liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      liftMotor.setPower(power);
   }

   public void setHand(double val) {
      handServo0.setPosition(INTAKE_HAND_0_POS - val);
      handServo1.setPosition(INTAKE_HAND_1_POS + val);
   }

   public void sleep_with_drive(double time_mm) {
      long start_time = System.currentTimeMillis();
      while (continue_condition.getAsBoolean() && System.currentTimeMillis() - start_time < time_mm) {
         drive_period.run();
      }
   }

   public void setIntakeArm(double pos) {
      intakeArmServo.setPosition(pos);
   }

   public void setLiftArm(double pos) {
      liftArmServo.setPosition(pos);
   }

   public void setIntakeMovePow(double pow) {
      intakeMoveMotor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intakeMoveMotor0.setPower(pow);
      intakeMoveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      intakeMoveMotor1.setPower(pow);
   }

   public void post_eject() {
      doorServo.setPosition(DOOR_RELEASE);
      setLifter(liftMotor.getCurrentPosition() + 30, 1);
      liftArmServo.setPosition(LIFT_ARM_POS_VERTICAL);
      sleep_with_drive(180);
      setLifter(LIFT_POS_MIN,1);
      liftArmServo.setPosition(LIFT_ARM_POS_TRANS);
   }

   public void internalConeSaveDeep() {
      coneSaveServo.setPosition(0.5);
      sleep_with_drive(180);
      coneSaveServo.setPosition(0.79);
   }

   public void internalConeSaveNear(){
      intakeArmServo.setPosition(INTAKE_ARM_POS_MIN);
      sleep_with_drive(200);
      setIntakeMovePositionPower(200,1);
      while (getIntakeMoveTicks()<175) drive_period.run();
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
   }

   public void grab(int time, double distanceDrag) {
      if (!continue_condition.getAsBoolean()) return;
      setHand(INTAKE_HAND_GRIP);
      setIntakeMovePow(0);
      sleep_with_drive(time);
      double startDistance = getIntakeMoveLength();
      while (startDistance - getIntakeMoveLength() < distanceDrag && getIntakeMoveTicks() > 10) {
         setIntakeMovePow(-0.8);
         drive_period.run();
      }
      setIntakeMovePow(0);
      intakeArmServo.setPosition(INTAKE_ARM_POS_VERTICAL);
      sleep_with_drive(100);
      setIntakeMovePositionPower(INTAKE_MOVE_WAY_POS, 1);
      drive_period.run();
   }
}
