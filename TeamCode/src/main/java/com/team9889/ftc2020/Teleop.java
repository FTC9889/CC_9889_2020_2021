package com.team9889.ftc2020;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuit;
import com.team9889.ftc2020.auto.actions.drive.DrivePurePursuitTeleOp;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.auto.actions.teleop.DriveAndShoot;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.lib.control.Path;
import com.team9889.lib.control.PurePursuit;
import com.team9889.lib.control.controllers.PID;

import java.util.ArrayList;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
public class Teleop extends Team9889Linear {

    private ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime(), wgTimer = new ElapsedTime();
    boolean on = false;

    double lastTime;
    boolean extend = false;

    boolean driveToPos = false, autoDrive = false;
    boolean driveFirst = true;
    Action drive;

    boolean lastWGState = false;

    ElapsedTime camTimer = new ElapsedTime();

    boolean resetPressed = false;

    private PID orientationPID = new PID(1, 0, 100);

    boolean psOn = false;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        wgTimer.reset();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > .5) {
                if (Robot.getCamera().currentCamState != Camera.CameraStates.GOAL) {
                    Robot.getCamera().setGoalCamPos();
                    camTimer.reset();
                }

                Robot.getCamera().setScanForGoal();

                if (camTimer.milliseconds() > 700) {

                    if (driveFirst) {
                        drive = new AimAndShoot();
                        drive.start();

                        driveFirst = false;
                    }
                    drive.update();

//                    driveToPos = true;
                }
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
                // Drive
                if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 || Math.abs(gamepad1.right_stick_x) > .1
                && gamepad1.right_trigger <= .5) {
                    Robot.getMecanumDrive().setFieldCentricPower(driverStation.getX() / driverStation.getSlowDownFactor(),
                            driverStation.getY() / driverStation.getSlowDownFactor(), driverStation.getSteer() / driverStation.getSlowDownFactor());
                } else if (!autoDrive && gamepad1.right_trigger <= .5) {
                                        Robot.getMecanumDrive().setFieldCentricPower(0, 0, 0);
                }
//                else if (driveToPos) {
//                    drive.update();
//
//                    if (drive.isFinished()) {
//                        drive.done();
//                        driveToPos = false;
//                    }
//                } else if (!autoDrive) {
//                    Robot.getMecanumDrive().setFieldCentricPower(0, 0, 0);
//                }

                if (driverStation.getStartIntaking()) {
                    Robot.getIntake().Intake();
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().Stop();
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().Outtake();
                }

                if (gamepad2.dpad_up) {
                    on = true;
                    psOn = true;
                } else if (gamepad2.dpad_right) {
                    on = false;
                    psOn = false;
                } else if (driverStation.getFW() && !psOn){
                    on = true;
                    psOn = false;
                } else if (!driverStation.getFW() && !psOn) {
                    on = false;
                    psOn = false;
                }

                if (!driveToPos) {
                    if (gamepad2.x) {
                        if (armTimer.milliseconds() > 300) {
                            if (extend) {
                                Robot.fwArm.setPosition(0.55);
                                extend = false;
                            } else {
                                Robot.fwArm.setPosition(0);
                                extend = true;
                            }

                            armTimer.reset();
                        }
                    } else if (gamepad1.right_bumper && on) {
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

                if (lastWGState != driverStation.getWG() || gamepad1.left_trigger > .5) {
                    wgTimer.reset();
                }

                if (autoDrive) {
                    Robot.wgLeft.setPosition(.2);
                    Robot.wgRight.setPosition(.2);
                } else if (!driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.wgGrabber.setPosition(0.25);
                    } else {
                        Robot.wgLeft.setPosition(.8);
                        Robot.wgRight.setPosition(.8);
                    }

                    lastWGState = false;
                } else if (driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.wgLeft.setPosition(0.4);
                        Robot.wgRight.setPosition(0.4);
                    } else {
                        Robot.wgGrabber.setPosition(.75);
                    }

                    lastWGState = true;
                }

                if (gamepad1.dpad_right || gamepad2.right_bumper){
//                    Robot.wgLeft.setPosition(.5);
//                    Robot.wgRight.setPosition(.5);
                    Robot.wgGrabber.setPosition(.75);
                }

                if (gamepad1.left_trigger > .5) {
                    if (Robot.getCamera().currentCamState != Camera.CameraStates.WG) {
                        Robot.getCamera().setWGCamPos();
                        camTimer.reset();
                    }
                    autoDrive = true;
                    Robot.getCamera().setScanForWG();

                    if (camTimer.milliseconds() > 700) {
                        if (Math.abs(driverStation.getX()) < 0.1 && Math.abs(driverStation.getY()) < 0.1 &&
                                Math.abs(driverStation.getSteer()) < 0.1) {

                            double forward = 0;
                            double turn = 0;

                            if (Robot.getCamera().getPosOfTarget().y > .5)
                                forward = .8;
                            else if (Robot.getCamera().getPosOfTarget().y <= .5)
                                forward = 0.5;

                            int center = 25;
                            if (Robot.getCamera().getPosOfTarget().x > .1)
                                turn = -0.2;
                            else if (Robot.getCamera().getPosOfTarget().x < -.1)
                                turn = 0.2;


                            Robot.getMecanumDrive().setPower(0, -forward, -turn);
                        }
                    }
                } else {
                    autoDrive = false;
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }


//            GAMEPAD 2
            if (gamepad2.a) {
                ArrayList<Path> pose = new ArrayList<>();
                pose.add(new Path(new Pose2d(47, 55, 0), new Pose2d(1, 1, 1), 8, 1, 5000));
                runAction(new DrivePurePursuitTeleOp(pose));
            } else if (gamepad2.b) {
                ArrayList<Path> pose = new ArrayList<>();
                pose.add(new Path(new Pose2d(57, 55, 0), new Pose2d(1, 1, 1), 8, 1, 5000));
                runAction(new DrivePurePursuitTeleOp(pose));
            } else if (gamepad2.y) {
                ArrayList<Path> pose = new ArrayList<>();
                pose.add(new Path(new Pose2d(67, 55, 0), new Pose2d(1, 1, 1), 8, 1, 5000));
                runAction(new DrivePurePursuitTeleOp(pose));
            }

            if (gamepad2.left_stick_button && gamepad2.right_stick_button && !resetPressed) {
                Robot.getMecanumDrive().setCurrentPose(new Pose2d(0, 0, 0));
                resetPressed = true;
            } else if (!gamepad2.left_stick_button && !gamepad2.right_stick_button) {
                resetPressed = false;
            }


            while (loopTimer.milliseconds() < 20) {

            }

            if (on) {
                if (!psOn)
                    Robot.getFlyWheel().setFlyWheelSpeed(5800, loopTimer.milliseconds());
                else
                    Robot.getFlyWheel().setFlyWheelSpeed(4500, loopTimer.milliseconds());
            }
            else if (!on) {
                Robot.flyWheel.setPower(0);
                Robot.getFlyWheel().wantedFWSpeed = 0;
                Robot.getFlyWheel().counter = 0;
                Robot.getFlyWheel().lastMotorPos = Robot.flyWheel.getPosition();
                Robot.getFlyWheel().pid.error_prior = 0;
                Robot.getFlyWheel().pid.first = true;
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());

            telemetry.addData("Front Encoder : ", Robot.intakeLeft.getPosition());
            telemetry.addData("Back Encoder : ", Robot.intakeRight.getPosition());
            telemetry.addData("Center Encoder : ", Robot.centerOdometry.getPosition());

            telemetry.addData("Odometry : ", Robot.getMecanumDrive().getCurrentPose());
            telemetry.addData("Odometry Adjusted : ", Robot.getMecanumDrive().getAdjustedPose());

            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            Robot.update();

            // dt timer
            loopTimer.reset();
        }
    }
}
