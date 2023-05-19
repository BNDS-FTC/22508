package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.*;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.XCYCode.Configurations;
@Autonomous
public class DefenceAutoRight extends BlueRightMiddleFast {
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
            } catch (Configurations.StructureJamException e) {
               intakeStuckSave();
            } catch (Configurations.TimeoutException e) {
               intakeSave();
               trans(false);
            }
            ejectFast(Junction.MIDDLE, 100);
         }
         park();
      } catch (Configurations.GlobalTimeoutException e) {
         park();
      } catch (Exception e) {
         throw new InterruptedException();
      } finally {
         savePosition();
      }
   }
}
