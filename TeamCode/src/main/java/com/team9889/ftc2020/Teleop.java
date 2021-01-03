package com.team9889.ftc2020;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.auto.actions.teleop.DriveAndShoot;
import com.team9889.lib.control.Path;
import com.team9889.lib.control.PurePursuit;

import java.util.ArrayList;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
public class Teleop extends Team9889Linear {

    private ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime();
    boolean on = false;

    double lastTime;
    boolean extend = false;

    boolean driveToPos = false;
    boolean driveFirst = true;
    Action drive;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        while (opModeIsActive()) {

            if (gamepad1.dpad_left) {
                if (driveFirst) {
//                    ArrayList<Path> path = new ArrayList<>();
//                    path.add(new Path(new Pose2d(0, 40, 0), new Pose2d(2, 2, 3), 8, 1, 5));
//                    drive = new DriveAndShoot(path);
                    drive = new AimAndShoot();
//                    drive.start();

                    driveFirst = false;
                }
                drive.start();
                driveToPos = true;
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
                // Drive
                if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1) {
                    Robot.getMecanumDrive().setFieldCentricPower(driverStation.getX() / driverStation.getSlowDownFactor(),
                            driverStation.getY() / driverStation.getSlowDownFactor(), driverStation.getSteer() / driverStation.getSlowDownFactor());
                } else if (driveToPos) {
                    drive.update();

                    if (drive.isFinished()) {
                        drive.done();
                        driveToPos = false;
                    }
                } else {
                    Robot.getMecanumDrive().setFieldCentricPower(0, 0, 0);
                }

                if (driverStation.getStartIntaking()) {
                    Robot.getIntake().Intake();
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().Stop();
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().Outtake();
                }

                if (gamepad1.x){
                    on = true;
                } else if (gamepad1.start) {
                    on = false;
                }

                if (!driveToPos) {
                    if (gamepad1.right_bumper && on) {
                        if (armTimer.milliseconds() > 300) {
                            if (extend) {
                                Robot.fwArm.setPosition(0.7);
                                extend = false;
                            } else {
                                Robot.fwArm.setPosition(0);
                                extend = true;
                            }

                            armTimer.reset();
                        }
                    } else if (armTimer.milliseconds() > 200) {
                        Robot.fwArm.setPosition(0);
                        extend = true;
                    }
                }

                if (!driverStation.getWG()) {
                    Robot.wgLeft.setPosition(.8);
                    Robot.wgRight.setPosition(.8);
                } else if (driverStation.getWG()) {
                    Robot.wgLeft.setPosition(0.4);
                    Robot.wgRight.setPosition(0.4);
                }

                if (driverStation.getWGG()) {
                    Robot.wgGrabber.setPosition(1);
                } else if (!driverStation.getWGG()) {
                    Robot.wgGrabber.setPosition(.6);
                }

                if (gamepad1.dpad_right){
                    Robot.wgLeft.setPosition(.5);
                    Robot.wgRight.setPosition(.5);
                    Robot.wgGrabber.setPosition(.3);
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }

            while (loopTimer.milliseconds() < 20) {

            }

            if (on) {
                Robot.getFlyWheel().setFlyWheelSpeed(5800, loopTimer.milliseconds());
            }
            else if (!on) {
                Robot.flyWheel.setPower(0);
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());

            telemetry.addData("Front Encoder : ", Robot.intakeLeft.getPosition());
            telemetry.addData("Back Encoder : ", Robot.intakeRight.getPosition());
            telemetry.addData("Center Encoder : ", Robot.centerOdometry.getPosition());

            telemetry.addData("Odometry : ", Robot.getMecanumDrive().getCurrentPose());
            telemetry.addData("Odometry Adjusted : ", Robot.getMecanumDrive().getAdjustedPose());
//            telemetry.addData("Last Angle : ", Robot.getMecanumDrive().odometry.returnLastAngle());

            // telemetry.addData("left intake", -Robot.intakeLeft.getPosition());
            // telemetry.addData("right intake", -Robot.intakeRight.getPosition());
            telemetry.addData("Fly Wheel", Robot.flyWheel.getPosition());
            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            Robot.update();

            // dt timer
            loopTimer.reset();
        }
    }
}
