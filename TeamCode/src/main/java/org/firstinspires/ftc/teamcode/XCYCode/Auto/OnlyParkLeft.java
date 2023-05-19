package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import org.firstinspires.ftc.teamcode.XCYCode.AutoMaster;

public class OnlyParkLeft extends AutoMaster {
   @Override
   public void runOpMode() throws InterruptedException{
      startSide=LEFT;
      initHardware();
      parkStart();
      savePosition();
   }
}
