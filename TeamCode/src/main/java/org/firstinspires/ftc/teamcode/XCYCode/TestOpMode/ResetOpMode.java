package org.firstinspires.ftc.teamcode.XCYCode.TestOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.XCYCode.Configurations;


@Autonomous(name = "reset all")
public class ResetOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx[] resetList = {hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.spin),
                hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.liftMotor),
                hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.intakeMove0),
                hardwareMap.get(DcMotorEx.class, Configurations.HardwareConstant.intakeMove1),
        };

        waitForStart();
        if (isStopRequested()) return;

        for (DcMotorEx motor:resetList){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

//        drive.followTrajectory(traj);
//        drive.setWeightedDrivePower(new Pose2d(0,0,0));
//        drive.waitForIdle();
//        sleep(2000);
    }
}
