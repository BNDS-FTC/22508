package org.firstinspires.ftc.teamcode.XCYCode;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class AutoMaster extends LinearOpMode {

   public enum Junction {
      SIDE, MIDDLE, EMPTY
   }

   private ElapsedTime runtime;
   public static final int RIGHT = -1;
   public static final int LEFT = 1;
   public static boolean DEBUG = true;
   public static int startTimeDelay = -1, cycleDelay =-1;

   public static final boolean RED = true;
   public static final boolean BLUE = false;

   protected Junction firstJunctionPos;
   protected int startSide;
   protected boolean side_color;
   protected boolean fastMode;

   public final static int LiftHighPos = 710;

   public static double x_axis = 1300, side_eject_y = 380;

   private int liftPos;

   private XCYMecanumDrive drive;
   private AutoSuperStructure upper;

   //   private static final double[] MIDDLE_GRAB_POS_ADD_MM = new double[]{30, 20, 5, 0, 15};
   private static final double[] MIDDLE_GRAB_POS_ADD_MM = new double[]{40, 55, 5, 0, 15};

   Pose2d[] RIGHT_END_POSITIONS = {
           new Pose2d(x_axis, -300, Math.toRadians(-90)),
           new Pose2d(x_axis, -300, Math.toRadians(-90)),
           new Pose2d(x_axis, -900, Math.toRadians(-90)),
           new Pose2d(x_axis, -1500, Math.toRadians(-90))
   };

   Pose2d[] LEFT_END_POSITIONS = {
           new Pose2d(x_axis, 300, Math.toRadians(90)),
           new Pose2d(x_axis, 1500, Math.toRadians(90)),
           new Pose2d(x_axis, 900, Math.toRadians(90)),
           new Pose2d(x_axis, 300, Math.toRadians(90))
   };
   Trajectory startToEject;

   Pose2d MIDDLE_EJECT_WAY_POS, MIDDLE_GRAB_POS, SIDE_POS, startPos;

   int end_pos_index;
   protected int cone_index;

   @Override
   public void runOpMode() throws InterruptedException {
      initHardware();
      try {
         cone_index = 4;
         longMoveNormal();
//         longMoveDefensive();
         cone_index = 5;
         if (fastMode)
            ejectFast(firstJunctionPos, 200);
         else
            ejectSlow(firstJunctionPos, 200);
         //         firstConeAttack();
//         intake(cone_index, Junction.SIDE);
///         trans();
//         ejectSlow(firstJunctionPos, 50);
         for (cone_index = 4; cone_index >= 0; cone_index--) {
            try {
               intake(cone_index, firstJunctionPos);
               trans();
//            } catch (ConeException e) {
//               intakeFalseSave();
//               cone_index++;
//               continue;
            } catch (StructureJamException e) {
               intakeStuckSave();
            } catch (TimeoutException e) {
               intakeSave();
               trans(false);
            }
            if (fastMode)
               ejectFast(firstJunctionPos, 100);
            else
               ejectSlow(firstJunctionPos, 100);
//            if (DEBUG) throw new InterruptedException();
         }
         park();
      } catch (GlobalTimeoutException e) {
         park();
      } catch (Exception e) {
         throw new InterruptedException();
      } finally {
         savePosition();
      }
   }

   protected void initHardware() throws InterruptedException {
      PhotonCore.enable();
      telemetry.addLine("init: webcam");
      telemetry.update();
      OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
              hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
      AutoDetectionPipeline pipeline = new AutoDetectionPipeline(0.045, 578.272, 578.272, 402.145, 221.506);
      webcam.setPipeline(pipeline);
      webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
         public void onOpened() {
            webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
         }

         @Override
         public void onError(int errorCode) {
         }
      });
      if (fastMode)
         MIDDLE_EJECT_WAY_POS = new Pose2d(x_axis, side_eject_y * startSide, Math.toRadians(90) * startSide).plus(new Pose2d(-60, -60 * startSide, 0));
      else
         MIDDLE_EJECT_WAY_POS = new Pose2d(x_axis, side_eject_y * startSide, Math.toRadians(90) * startSide);

      MIDDLE_GRAB_POS = new Pose2d(x_axis, 880 * startSide, Math.toRadians(90) * startSide);
      SIDE_POS = new Pose2d(x_axis, 995 * startSide, Math.toRadians(91.7) * startSide);
      startPos = new Pose2d(0, 900 * startSide, Math.toRadians(180) * startSide);
      end_pos_index = 0;
      FtcDashboard.getInstance().startCameraStream(webcam, 10);
//        telemetry.setMsTransmissionInterval(200);
      telemetry.update();
      telemetry.addLine("init: drive");
      drive = new XCYMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      telemetry.addLine("init: superstructure");
      telemetry.update();
      upper = new AutoSuperStructure(
              hardwareMap,
              this::opModeIsActive,
              drive::update
      );
      upper.setSideIsRed(side_color);
      telemetry.addLine("init: trajectory");
      telemetry.update();
      if (firstJunctionPos == Junction.MIDDLE) {
         if (fastMode)
            startToEject = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(900, 900 * startSide, Math.toRadians(180) * startSide))
                    .splineToSplineHeading(new Pose2d(x_axis, 600 * startSide, Math.toRadians(90) * startSide), Math.toRadians(-90) * startSide)
                    .splineToSplineHeading(MIDDLE_EJECT_WAY_POS, Math.toRadians(-90) * startSide)
                    .build();
         else
            startToEject = drive.trajectoryBuilder(startPos)
                    .lineToLinearHeading(new Pose2d(900, 900 * startSide, Math.toRadians(180) * startSide))
                    .splineToSplineHeading(new Pose2d(x_axis, 600 * startSide, Math.toRadians(90) * startSide), Math.toRadians(-90) * startSide)
                    .splineToSplineHeading(MIDDLE_EJECT_WAY_POS, Math.toRadians(-90) * startSide)
                    .build();
      } else {
         startToEject = drive.trajectoryBuilder(startPos)
                 .lineToSplineHeading(SIDE_POS)
                 .build();
      }
      telemetry.addLine("init: pipeline");
      telemetry.update();
      upper.setIntakeArm(INTAKE_ARM_POS_VERTICAL - 0.05);
      upper.setLiftArm(LIFT_ARM_POS_HOLD);
      upper.junctionAimServoBack();
      upper.doorServo.setPosition(DOOR_RELEASE);
      runtime = new ElapsedTime();
      runtime.reset();
      upper.setGlobalTime(() -> runtime.seconds() > 28);

      while (!opModeIsActive()) {
         int id = pipeline.getId();
         end_pos_index = id == 0 ? end_pos_index : id;
         long time = System.currentTimeMillis();
         telemetry.addData("pos", end_pos_index);
         telemetry.update();
         sleep(15);
         while (System.currentTimeMillis() - time < 100 && opModeInInit()) idle();
         if (isStopRequested()) throw new InterruptedException();
      }
      waitForStart();
      runtime.reset();
      upper.doorServo.setPosition(DOOR_HOLD);
      webcam.closeCameraDeviceAsync(() -> {
      });
   }

   protected void longMoveNormal() throws Exception{
      if (isStopRequested()) return;
      liftPos = firstJunctionPos == Junction.MIDDLE ? XCYSuperStructure.HIGH_LIFT_MIN - 10 : LiftHighPos;
      upper.setHand(0);
      drive.followTrajectoryAsync(startToEject);
      if (firstJunctionPos == Junction.MIDDLE) {
         upper.setDrivePeriod(drive::simpleMovePeriod);
         while (opModeIsActive() && Math.abs(drive.getPoseEstimate().getY() - (400 * startSide)) > 100) {
            upper.wait_for_trans_update();
            drive.update();
         }
         drive.initSimpleMove(MIDDLE_EJECT_WAY_POS);
         while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
            drive.simpleMovePeriod(0.95);
         }
      } else {
         while (opModeIsActive() && drive.getPoseEstimate().getX() < 900) {
            upper.wait_for_trans_update();
            drive.update();
         }
      }
      upper.set_eject_spin_pos(startSide * (firstJunctionPos == Junction.MIDDLE ? -SPIN_AMPLITUDE : SPIN_AMPLITUDE));
      upper.sleep_with_drive(startTimeDelay);
      if (fastMode) {
         upper.eject_ready(710, false, INTAKE_ARM_POSITIONS[cone_index]);
         upper.junctionAimServoOut();
         upper.setDrivePeriod(drive::simpleMovePeriod);
      } else {
         upper.eject_ready(liftPos, false, INTAKE_ARM_POSITIONS[cone_index]);
         while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            if (isStopRequested()) throw new InterruptedException();
         }
      }
   }

   //到位等待，进行后面动作
   protected void longMoveDefensive() throws Exception {
      upper.setDrivePeriod(drive::simpleMovePeriod);
      drive.initSimpleMove(new Pose2d(1410, startPos.getY(), startPos.getHeading()));
      while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
         drive.simpleMovePeriod(0.95);
      }
      drive.initSimpleMove(MIDDLE_EJECT_WAY_POS);
      while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
         drive.simpleMovePeriod(0.95);
      }
      upper.set_eject_spin_pos(startSide * (firstJunctionPos == Junction.MIDDLE ? -SPIN_AMPLITUDE : SPIN_AMPLITUDE));
      drive.initSimpleMove(MIDDLE_EJECT_WAY_POS);
      upper.setDrivePeriod(()->drive.simpleMovePeriod(0.7));
      upper.sleep_with_drive(500);
      upper.eject_ready(710, false, INTAKE_ARM_POSITIONS[cone_index]);
      upper.junctionAimServoOut();
      upper.setDrivePeriod(drive::simpleMovePeriod);
   }

   Pose2d SIDE_ATTACK_POS;

   //第一个桶，往回拉中立高杆，结束点近
   protected void firstConeAttack() throws Exception {
      SIDE_ATTACK_POS = new Pose2d(x_axis, startPos.getY(), startPos.getHeading());

      upper.set_eject_spin_pos(-SPIN_AMPLITUDE * startSide);
      drive.initSimpleMove(SIDE_ATTACK_POS);
      upper.setDrivePeriod(() -> drive.simpleMovePeriod(1));

      while (opModeIsActive() && drive.getPoseEstimate().getX() < 500) {
         upper.wait_for_trans_update();
         drive.simpleMovePeriod(1);
      }

      drive.initSimpleMove(new Pose2d(x_axis, 790 * startSide, startPos.getHeading()));

      upper.eject_ready(710, false, INTAKE_ARM_POS_VERTICAL);
      while (opModeIsActive() && drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
         drive.simpleMovePeriod(0.95);
      }
      upper.sleep_with_drive(100);
      upper.preEjectAction(0.11);
      upper.setLifter(XCYSuperStructure.HIGH_LIFT_MIN, 1);
      upper.set_eject_spin_pos(150 * startSide);
      drive.initSimpleMove(SIDE_POS);
      upper.junctionAimServoBack();
      double pos_err = drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm();
      while (opModeIsActive() && pos_err > 30) {
         upper.setSpinPosition(150 * startSide);
         drive.simpleMovePeriod(1);
         pos_err = drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm();
      }
      upper.sleep_with_drive(150);
      upper.setIntakeMoveLength(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE + 470 + MIDDLE_GRAB_POS_ADD_MM[cone_index]);
      upper.sleep_with_drive(150);
      upper.eject_action(0.10, 100, 50);
      upper.setSpinPosition(SPIN_MID);
      upper.post_eject();
   }


   protected void intake(int index, Junction lastJunction) throws Exception {
      upper.setTransLiftPos(LIFT_POS_MIN);
      upper.setDrivePeriod(drive::simpleMovePeriod);
      if (lastJunction == Junction.SIDE) {
         drive.initSimpleMove(SIDE_POS);
         upper.toAim(INTAKE_ARM_POSITIONS[index]);
         while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
            drive.simpleMovePeriod(0.95);
            if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
         }
         upper.expand(INTAKE_ARM_POSITIONS[index], XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE + 650 + MIDDLE_GRAB_POS_ADD_MM[cone_index]);
      } else {
         drive.initSimpleMove(MIDDLE_GRAB_POS.plus(new Pose2d(0, MIDDLE_GRAB_POS_ADD_MM[index] * startSide, 0)));
         upper.toAim(INTAKE_ARM_POSITIONS[index]);
         if (!fastMode)
            upper.sleep_with_drive(250);
         else {
            upper.sleep_with_drive(150);
            upper.post_eject();
         }
         upper.setIntakeMoveLength(AutoSuperStructure.MAX_INTAKE_MOVE_DISTANCE - AutoSuperStructure.intakeSlowRange);
         while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
            drive.simpleMovePeriod(0.95);
            if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
         }
         upper.expand(INTAKE_ARM_POSITIONS[index], AutoSuperStructure.MAX_INTAKE_MOVE_DISTANCE);
      }
      if (cone_index < 2)
         upper.grab(80, 4);
      else upper.grab(90, 0);
//      if (!upper.checkConeValid()) throw new ConeException();
   }

   protected void trans() throws Exception {
      trans(cone_index < 2);
   }

   protected void trans(boolean transLow) throws Exception {

      drive.initSimpleMove(firstJunctionPos == Junction.MIDDLE ? MIDDLE_EJECT_WAY_POS : SIDE_POS);
      upper.setDrivePeriod(() -> drive.simpleMovePeriod(0.5));

      if (transLow) upper.transLow();
      else upper.transHigh();

      upper.set_eject_spin_pos(startSide * (firstJunctionPos == Junction.MIDDLE ? -SPIN_AMPLITUDE : SPIN_AMPLITUDE));
      if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
      upper.sleep_with_drive(cycleDelay);

      if (firstJunctionPos == Junction.MIDDLE) {
         if (fastMode) {
            upper.eject_ready(LiftHighPos, false, INTAKE_ARM_POSITIONS[cone_index]);
            upper.junctionAimServoOut();
         }
      } else if (firstJunctionPos == Junction.SIDE) {
         upper.eject_ready(LiftHighPos, false, INTAKE_ARM_POSITIONS[cone_index]);
      }
      while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
         drive.simpleMovePeriod(0.5);
      }
   }

   protected void ejectSlow(Junction pos, int stable_time) throws Exception {
      liftPos = firstJunctionPos == Junction.MIDDLE ? XCYSuperStructure.HIGH_LIFT_MIN - 10 : LiftHighPos;

      if (runtime.seconds() > 28.8) throw new GlobalTimeoutException();
//      drive.initSimpleMove(pos == Junction.MIDDLE ? MIDDLE_EJECT_POS : SIDE_POS);
      upper.setDrivePeriod(drive::simpleMovePeriod);
      while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
         drive.simpleMovePeriod(0.95);
      }
      if (pos == Junction.SIDE && cone_index > 0) {
         upper.intakeArmServo.setPosition(INTAKE_ARM_POSITIONS[cone_index - 1]);
         upper.setIntakeMoveLength(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE
                 + 450
                 + MIDDLE_GRAB_POS_ADD_MM[cone_index - 1]
                 - AutoSuperStructure.intakeSlowRange);
      }
      upper.sleep_with_drive(stable_time);
      upper.preEjectAction(0.11);

      upper.eject_action(0.11, 150);
      upper.post_eject();
      upper.sleep_with_drive(150);
      if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
      upper.setSpinPosition(SPIN_MID);
   }

   protected void ejectFast(Junction pos, int stable_time) throws Exception {
      if (runtime.seconds() > 28.8) throw new GlobalTimeoutException();
      drive.initSimpleMove(pos == Junction.MIDDLE ? MIDDLE_EJECT_WAY_POS : SIDE_POS);
      upper.setDrivePeriod(drive::simpleMovePeriod);
      if (pos == Junction.SIDE && cone_index > 0) {
         upper.intakeArmServo.setPosition(INTAKE_ARM_POSITIONS[cone_index - 1]);
         upper.setIntakeMoveLength(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE
                 + 450
                 + MIDDLE_GRAB_POS_ADD_MM[cone_index - 1]
                 - AutoSuperStructure.intakeSlowRange);
      }
      while (drive.getSimpleMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
         drive.simpleMovePeriod(1);
      }
      upper.sleep_with_drive(stable_time);
      upper.eject_action(0.03, 350, 0);
      upper.junctionAimServoBack();
      if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
      upper.setSpinPosition(SPIN_MID);
   }

   protected void intakeStuckSave() throws Exception {
      upper.setIntakeMoveLength(XCYSuperStructure.MIN_INTAKE_MOVE_DISTANCE + 150);
      upper.setDrivePeriod(drive::simpleMovePeriod);
      drive.initSimpleMove(new Pose2d(drive.getPoseEstimate().vec(), Math.toRadians(45) * startSide));
      upper.sleep_with_drive(200);
      drive.initSimpleMove(new Pose2d(drive.getPoseEstimate().vec(), Math.toRadians(90 + 45) * startSide));
      upper.sleep_with_drive(200);
      drive.initSimpleMove(new Pose2d(drive.getPoseEstimate().vec(), Math.toRadians(90) * startSide));
   }

   protected void intakeSave() throws Exception {
//      drive.initPreciseMove(drive.getPoseEstimate().plus(new Pose2d(0,-150*startSide,0)));
//      upper.setIntakeMovePositionPower(0, 0.6);
//      while (drive.getPreciseMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
//         drive.preciseMovePeriod(0.95);
//         if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
//      }
//      drive.initPreciseMove(SIDE_POS);
//      upper.setIntakeMovePositionPower(INTAKE_MOVE_MAX_POS, 0.4);
//      while (drive.getPreciseMovePosition().minus(drive.getPoseEstimate()).vec().norm() > 30) {
//         drive.preciseMovePeriod(0.95);
//         if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
//      }
//      long start = System.currentTimeMillis();
//      while (!upper.isConeDetected()) {
//         drive.preciseMovePeriod();
//         if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
//      }
      upper.setIntakeMovePow(0.3);
      upper.sleep_with_drive(200);
      upper.grab(120, 0);
   }

   private void intakeFalseSave() throws Exception {
      upper.toAim();
      Pose2d pose2d = drive.getPoseEstimate();
      drive.initSimpleMove(pose2d.plus(new Pose2d(0, 0, Math.toRadians(20 * startSide))));
      upper.sleep_with_drive(400);
      drive.initSimpleMove(pose2d);
      upper.sleep_with_drive(400);
   }

   //倾覆
   private void tiltSave() {
      drive.setDrivePower(new Pose2d());
      drive.stopTrajectory();
      upper.setIntakeMovePow(0);
      drive.update();
      Thread.currentThread().interrupt();
   }

   //停靠，起始位置x=1380
   protected void park() {
      Pose2d endPos;
      if (startSide == -1) {
         endPos = (RIGHT_END_POSITIONS[end_pos_index]);
      } else {
         endPos = (LEFT_END_POSITIONS[end_pos_index]);
      }
      upper.setDrivePeriod(drive::simpleMovePeriod);
      drive.initSimpleMove(endPos);
      upper.setTransLiftPos(0);
      upper.setIntakeMovePositionPower(0, 0.8);
      upper.setLiftArm(LIFT_ARM_POS_VERTICAL);
      upper.setLifter(0, 0.5);
      upper.sleep_with_drive(150);
      upper.setSpinPosition(0);
      upper.setIntakeArm(INTAKE_ARM_POS_VERTICAL);
      while (drive.getPoseEstimate().minus(endPos).vec().norm() > 30) {
         drive.simpleMovePeriod();
         upper.wait_for_trans_update();
      }
      upper.sleep_with_drive(200);
   }

   protected void parkStart(){
      Pose2d endPos;
      if (startSide == -1) {
         endPos = (RIGHT_END_POSITIONS[end_pos_index]);
      } else {
         endPos = (LEFT_END_POSITIONS[end_pos_index]);
      }
      upper.setDrivePeriod(drive::simpleMovePeriod);
      drive.initSimpleMove(endPos.plus(new Pose2d(-1200,0,0)));

      while (drive.getPoseEstimate().minus(endPos).vec().norm() > 30) {
         drive.simpleMovePeriod();
         upper.wait_for_trans_update();
      }
      upper.sleep_with_drive(200);
   }

   protected void savePosition(){
      save_pos_in_csv(drive.getPoseEstimate());
   }
}
