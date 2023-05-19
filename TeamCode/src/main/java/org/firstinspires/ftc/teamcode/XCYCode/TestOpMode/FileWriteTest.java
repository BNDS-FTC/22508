package org.firstinspires.ftc.teamcode.XCYCode.TestOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.XCYCode.Configurations;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;

@TeleOp(name = "read write file")
@Config
public class FileWriteTest extends LinearOpMode {
    public static double x_start=0;
    public static double y_start=0;
    public static double heading_start=0;
    public static boolean write = false;
    public static boolean read = true;

    @Override
    public void runOpMode() {
        Telemetry telemetry_M = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        if (write)
            Configurations.save_pos_in_csv(new Pose2d(x_start,y_start,Math.toRadians(heading_start)));
        if (read) {
            Pose2d pos = Configurations.get_pos_from_csv();
            telemetry_M.addLine(pos.toString());
        }
        waitForStart();
        telemetry_M.update();
        while (opModeIsActive()) {
            idle();
        }
    }
}
