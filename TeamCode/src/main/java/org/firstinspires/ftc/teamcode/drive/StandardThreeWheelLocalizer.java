package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.XCYCode.Configurations;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

@Config
public class StandardThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
   public static double TICKS_PER_REV = 4096;
   public static double WHEEL_RADIUS = 25; // in
   public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

   public static double LATERAL_DISTANCE = 151.5; // in; distance between the left and right wheels
   public static double FORWARD_OFFSET = 56.75; // in; offset of the lateral wheel

   private Encoder leftEncoder, rightEncoder, frontEncoder;

   private List<Integer> lastEncPositions= new ArrayList<>(), lastEncVels=new ArrayList<>();

   public StandardThreeWheelLocalizer(HardwareMap hardwareMap) {
      super(Arrays.asList(
              new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
              new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
              new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
      ));

      leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.leftEncoderName));
      rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.rightEncoderName));
      frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.frontEncoderName));

      leftEncoder.setDirection(Configurations.HardwareConstant.leftEncoderDirection);
      rightEncoder.setDirection(Configurations.HardwareConstant.rightEncoderDirection);
      frontEncoder.setDirection(Configurations.HardwareConstant.frontEncoderDirection);
      getWheelPositions();
      getWheelVelocities();
      // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
   }

   public static double encoderTicksToInches(double ticks) {
      return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
   }

   @NonNull
   @Override
   public List<Double> getWheelPositions() {
      int leftPos = leftEncoder.getCurrentPosition();
      int rightPos = rightEncoder.getCurrentPosition();
      int frontPos = frontEncoder.getCurrentPosition();

      lastEncPositions.clear();
      lastEncPositions.add(leftPos);
      lastEncPositions.add(rightPos);
      lastEncPositions.add(frontPos);

      return Arrays.asList(
              encoderTicksToInches(leftPos),
              encoderTicksToInches(rightPos),
              encoderTicksToInches(frontPos)
      );
   }

   @NonNull
   @Override
   public List<Double> getWheelVelocities() {
      int leftVel = (int) leftEncoder.getCorrectedVelocity();
      int rightVel = (int) rightEncoder.getCorrectedVelocity();
      int frontVel = (int) frontEncoder.getCorrectedVelocity();

      lastEncVels.clear();
      lastEncVels.add(leftVel);
      lastEncVels.add(rightVel);
      lastEncVels.add(frontVel);

      return Arrays.asList(
              encoderTicksToInches(leftVel),
              encoderTicksToInches(rightVel),
              encoderTicksToInches(frontVel)
      );
   }
}