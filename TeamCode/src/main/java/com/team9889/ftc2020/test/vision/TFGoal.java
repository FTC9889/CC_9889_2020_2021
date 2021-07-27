package com.team9889.ftc2020.test.vision;

import android.graphics.RectF;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.Team9889Linear;
import com.team9889.ftc2020.auto.actions.teleop.FindGoal;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by Eric on 7/5/2021.
 */

@TeleOp
@Config
public class TFGoal extends Team9889Linear {
    public static double widthOfGoal = 24, f = 432.08;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart(false);

        timer.reset();

        Robot.camera.setPipeline(Robot.getCamera().scanForGoal);
        ThreadAction(new FindGoal());

        while (opModeIsActive()) {
            telemetry.addData("Loop Time", timer.milliseconds());
            telemetry.addData("Gyro", Robot.getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES));
            if (Robot.getCamera().scanForGoal.goal != null) {
                RectF bb = Robot.getCamera().scanForGoal.goal.getBoundingBox();

                float p = bb.right - bb.left;
                double dist = ((widthOfGoal * f) / p);
                telemetry.addData("Goal Position", dist);

                double goalPos = -(((bb.centerX()) / 160) - 1);
                double fovAngle = goalPos * (53.4 / 2) * .625;
                double fullAngle = fovAngle + Robot.getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES);
                telemetry.addData("Goal Pos", goalPos);
                telemetry.addData("FOV Angle", fovAngle);
                telemetry.addData("Cam to Goal Angle", fullAngle);

                telemetry.addData("Width", Math.sin(Math.toRadians(fullAngle)) * dist);
                telemetry.addData("Height", Math.cos(Math.toRadians(fullAngle)) * dist);
            }
            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            Robot.update();
            timer.reset();
        }
    }
}
