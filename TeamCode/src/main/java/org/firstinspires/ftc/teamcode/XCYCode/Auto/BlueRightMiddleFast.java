package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.XCYCode.AutoMaster;

@Autonomous
public class BlueRightMiddleFast extends AutoMaster {
   public BlueRightMiddleFast(){
      startSide = RIGHT;
      side_color = BLUE;
      fastMode = true;
      firstJunctionPos = Junction.MIDDLE;
   }
}
