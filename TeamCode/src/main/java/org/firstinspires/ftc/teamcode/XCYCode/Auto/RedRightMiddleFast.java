package org.firstinspires.ftc.teamcode.XCYCode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.XCYCode.AutoMaster;

@Autonomous
public class RedRightMiddleFast extends AutoMaster {
   public RedRightMiddleFast(){
      startSide = RIGHT;
      side_color = RED;
      fastMode = true;
      firstJunctionPos = Junction.MIDDLE;
   }
}
