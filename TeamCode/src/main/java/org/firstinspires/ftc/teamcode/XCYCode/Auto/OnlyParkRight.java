package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import org.firstinspires.ftc.teamcode.XCYCode.AutoMaster;

public class OnlyParkRight extends AutoMaster {
   @Override
   public void runOpMode() throws InterruptedException{
      startSide=RIGHT;
      initHardware();
      parkStart();
      savePosition();
   }
}
