package com.team9889.ftc2020.test.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.Team9889Linear;
import com.team9889.ftc2020.auto.actions.utl.RobotUpdate;
import com.team9889.lib.control.Path;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

/**
 * Created by Eric on 2/7/2020.
 */

@TeleOp
@Disabled
public class TunePurePursuitPID extends Team9889Linear {
    ArrayList<Path> path = new ArrayList<>();
    double p = 0.03, i = 0, d = 0.3;
    double maxVelocity = 1;
    int number = 1;

    ElapsedTime loopTimer = new ElapsedTime();

    boolean lRToggle = true, upDownToggle = true, speedToggle = true;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart(false);

        ThreadAction(new RobotUpdate());

        while (opModeIsActive()) {
//            Robot.update();

            PID pid = new PID(p, i, d);

            if (gamepad1.a) {
                path.add(new Path(new Pose2d(
                        1,
                        0,
                        Robot.getMecanumDrive().gyroAngle.getTheda(AngleUnit.DEGREES) + 90),
                        new Pose2d(1, 1, 2), 30, maxVelocity));

//                runAction(new DrivePurePursuit(path, pid));
                path.clear();
            }

            if (gamepad1.dpad_left && lRToggle) {
                number--;
                lRToggle = false;
            } else if (gamepad1.dpad_right && lRToggle) {
                number++;
                lRToggle = false;
            } else if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
                lRToggle = true;
            }

            if (gamepad1.dpad_up && upDownToggle) {
                upDownToggle = false;
                if (number < 2)
                    p += .001;
                else if (number == 2)
                    i += .001;
                else if (number > 2)
                    d += .1;
            } else if (gamepad1.dpad_down && upDownToggle) {
                upDownToggle = false;
                if (number < 2)
                    p -= .001;
                else if (number == 2)
                    i -= .001;
                else if (number > 2)
                    d -= .001;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                upDownToggle = true;
            }

            if (gamepad1.left_bumper && speedToggle) {
                speedToggle = false;
                maxVelocity -= .1;
            } else if (gamepad1.right_bumper && speedToggle) {
                speedToggle = false;
                maxVelocity += .1;
            } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                speedToggle = true;
            }

            while (loopTimer.milliseconds() < 20) {

            }

            if (number < 2)
                telemetry.addData("PID", "[" + p + "]" + ", " + i + ", " + d);
            else if (number == 2)
                telemetry.addData("PID", p + ", " + "[" + i + "]" + ", " + d);
            else if (number > 2)
                telemetry.addData("PID", p + ", " + i + ", " + "[" + d + "]");

//            telemetry.addData("X", Robot.getMecanumDrive().getCurrentPose().getX());

            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addLine();
            Robot.outputToTelemetry(telemetry);

            telemetry.update();
            loopTimer.reset();
        }
    }
}
