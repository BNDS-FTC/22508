package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.GlobalTimeoutException;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.StructureJamException;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.TimeoutException;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedDefenceAutoLeft extends RedLeftMiddleFast {
   @Override
   public void runOpMode() throws InterruptedException {
      initHardware();
      try {
         cone_index = 4;
         longMoveDefensive();
         cone_index = 5;
         ejectFast(Junction.MIDDLE, 200);
         for (cone_index = 4; cone_index >= 0; cone_index--) {
            try {
               intake(cone_index, Junction.MIDDLE);
               trans();
            } catch (StructureJamException e) {
               intakeStuckSave();
            } catch (TimeoutException e) {
               intakeSave();
               trans(false);
            }
            ejectFast(Junction.MIDDLE, 100);
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
}
