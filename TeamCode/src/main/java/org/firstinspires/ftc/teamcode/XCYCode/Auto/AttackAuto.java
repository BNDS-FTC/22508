package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.GlobalTimeoutException;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.StructureJamException;
import static org.firstinspires.ftc.teamcode.XCYCode.Configurations.TimeoutException;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class AttackAuto extends BlueRightMiddleFast {
   @Override
   public void runOpMode() throws InterruptedException {
      initHardware();
      try {
         firstConeAttack();
         if (DEBUG) throw new InterruptedException();
         intake(4,Junction.SIDE);
         trans(false);
         ejectFast(Junction.MIDDLE,300);
         for (cone_index = 3; cone_index >= 0; cone_index--) {
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
