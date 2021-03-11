package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MannoMation on 1/14/2019.
 */

@Config
@TeleOp
public class Teleop extends Team9889Linear {
    public static double x = 103, y = 42, ay = 19.58, ax = 2.603092035700193;

    private ElapsedTime loopTimer = new ElapsedTime(), readyTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime(), wgTimer = new ElapsedTime(), wgAutoTimer = new ElapsedTime();
    boolean on = false;

    double lastTime;
    boolean extend = false;

    boolean driveToPos = false, autoDrive = false;
    boolean driveFirst = true, turnFirst = true, psFirst = true;
    Action drive, turn, ps;

    boolean lastWGState = false;

    ElapsedTime camTimer = new ElapsedTime();

    private PID orientationPID = new PID(1, 0, 100);

    public static double rpm = 2540;

    boolean psOn = false;
    boolean wgInPos = false;
    boolean wgFirst = true;
    boolean autoAimreleased = false;
    int readyCount = 0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        wgTimer.reset();
        Robot.flyWheel.resetEncoder();
        Robot.getFlyWheel().turnOffFlyWheel();

        while (opModeIsActive()) {
            double dist = (0.051 * Math.pow(Robot.getCamera().scanForGoal.getPointInPixels().y, 2))
                    - (3.0635 * Robot.getCamera().scanForGoal.getPointInPixels().y) + 117.19;

            if (gamepad2.right_trigger > .1 || gamepad2.a || gamepad2.b || gamepad2.y) {
                rpm = (0.0958 * Math.pow(dist, 2)) - (13.478 * dist) + 3008.7;

                if (Robot.getCamera().currentCamState != Camera.CameraStates.GOAL) {
                    Robot.getCamera().camYPose = .7;
                    Robot.getCamera().setGoalCamPos();
                    camTimer.reset();
                }

                if (gamepad2.a) {
                    Robot.getCamera().setPS1CamPos();
                    rpm -= 100;
                } else if (gamepad2.b) {
                    Robot.getCamera().setPS2CamPos();
                    rpm -= 100;
                } else if (gamepad2.y){
                    Robot.getCamera().setPS3CamPos();
                    rpm -= 100;
                } else if (gamepad2.right_trigger > .1) {
                    Robot.getCamera().setGoalCamPos();
                }

                Robot.getCamera().setScanForGoal();

                if (camTimer.milliseconds() > 700) {

                    if (driveFirst) {
                        drive = new AimAndShoot();
                        drive.start();

                        driveFirst = false;
                    }
                    drive.update();
                }

                if (Math.abs(Robot.getCamera().getPosOfTarget().x) < .1) {
                    if (readyCount >= 3) {
                        Robot.getWobbleGoal().open();
                    } else {
                        Robot.getWobbleGoal().close();
                    }
                    readyCount++;
                } else {
                    readyCount = 0;
                }

                autoAimreleased = true;
            } else if (autoAimreleased) {
                Robot.getWobbleGoal().close();
                autoAimreleased = false;
            }
//            else if (gamepad2.a || gamepad2.b || gamepad2.y) {
//                rpm = (0.0958 * Math.pow(dist, 2)) - (13.478 * dist) + 3008.7;
//
//                if (Robot.getCamera().currentCamState != Camera.CameraStates.GOAL) {
//                    Robot.getCamera().camYPose = .7;
//                    Robot.getCamera().setGoalCamPos();
//                    camTimer.reset();
//                }
//
//                if (gamepad2.a) {
//                    Robot.getCamera().setPS1CamPos();
//                } else if (gamepad2.b) {
//                    Robot.getCamera().setPS2CamPos();
//                } else if (gamepad2.y){
//                    Robot.getCamera().setPS3CamPos();
//                }
//
//                Robot.getCamera().setScanForGoal();
//
//                if (camTimer.milliseconds() > 700) {
//
//                    if (psFirst) {
//                        ps = new AimAndShoot();
//                        ps.start();
//
//                        psFirst = false;
//                    }
//                    ps.update();
//                }
//            }


            // If not resetting imu, normal operation
            if(!driverStation.resetIMU() && !driverStation.resetRobotPose()) {

                // Drive
                if (Math.abs(gamepad1.left_stick_x) > .1 || Math.abs(gamepad1.left_stick_y) > .1 ||
                        Math.abs(gamepad1.right_stick_x) > .1 && gamepad1.right_trigger <= .5) {
                    Robot.getMecanumDrive().xSpeed += driverStation.getX() / driverStation.getSlowDownFactor();
                    Robot.getMecanumDrive().ySpeed += driverStation.getY() / driverStation.getSlowDownFactor();
                    Robot.getMecanumDrive().turnSpeed += driverStation.getSteer() / driverStation.getSlowDownFactor();
                }

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
                } else if (driverStation.getFW() && (!psOn || gamepad1.x)){
                    on = true;
                    psOn = false;
                } else if (!driverStation.getFW() && (!psOn || gamepad1.x)) {
                    on = false;
                    psOn = false;
                }

                // Fire control
                if (!driveToPos) {
                    if (gamepad2.x) {
                        if (armTimer.milliseconds() > 135) {
                            if (extend) {
                                Robot.getFlyWheel().extend();
                                extend = false;
                            } else {
                                Robot.getFlyWheel().retract();
                                extend = true;
                            }

                            armTimer.reset();
                        }
                    } else if (gamepad1.right_bumper && on) {
                        if (armTimer.milliseconds() > 135) {
                            if (extend) {
                                Robot.getFlyWheel().extend();
                                extend = false;
                            } else {
                                Robot.getFlyWheel().retract();
                                extend = true;
                            }

                            armTimer.reset();
                        }
                    } else if (armTimer.milliseconds() > 135) {
                        Robot.getFlyWheel().extend();
                        extend = false;
                    }
                }

                if (lastWGState != driverStation.getWG() || gamepad1.left_trigger > .5) {
                    wgTimer.reset();
                }
                if (!wgInPos) {
                    wgAutoTimer.reset();
                } else if (gamepad1.left_bumper){
                    wgInPos = false;
                    driverStation.codePress = true;
                }

                if (autoDrive && !wgInPos) {
                    Robot.getWobbleGoal().inBetween();
                } else if (wgInPos) {
                    if (wgAutoTimer.milliseconds() < 500) {
                        Robot.getWobbleGoal().down();
                    } else if (wgAutoTimer.milliseconds() > 500 && wgAutoTimer.milliseconds() < 1000) {
                        Robot.getWobbleGoal().close();
                    } else {
                        Robot.getWobbleGoal().up();
                    }
                } else if (!driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.getWobbleGoal().close();
                    } else {
                        Robot.getWobbleGoal().up();
                    }

                    lastWGState = false;
                } else if (driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.getWobbleGoal().down();
                    } else {
                        Robot.getWobbleGoal().open();
                    }

                    lastWGState = true;
                }

                if (gamepad1.dpad_right || gamepad2.right_bumper){
                    // Robot.wgLeft.setPosition(.5);
                    // Robot.wgRight.setPosition(.5);
                    Robot.getWobbleGoal().open();
                }

                if (gamepad1.left_trigger > .5 && !wgInPos) {
                    if (wgFirst) {
                        Robot.getCamera().setWGCamPos();
                        camTimer.reset();
                        wgFirst = false;
                    }
                    autoDrive = true;
                    Robot.getCamera().setScanForWG();

                    if (camTimer.milliseconds() > 700) {
                        if (Math.abs(driverStation.getX()) < 0.1 && Math.abs(driverStation.getY()) < 0.1 &&
                                Math.abs(driverStation.getSteer()) < 0.1) {

                            double forward = 0.5;
                            double turn = 0;

//                            if (Robot.getCamera().getPosOfTarget().y > .5)
//                                forward = .5;
//                            else if (Robot.getCamera().getPosOfTarget().y <= .5)
//                                forward = 0.5;

                            int center = 25;
                            if (Robot.getCamera().getPosOfTarget().x > .1)
                                turn = -0.1;
                            else if (Robot.getCamera().getPosOfTarget().x < -.1)
                                turn = 0.1;


                            Robot.getMecanumDrive().setPower(0, -forward, -turn);
                        }
                    }

                    if (Robot.wobbleGoalRangeFinder.getDistance(DistanceUnit.INCH) < 3.5) {
                        wgInPos = true;
                        wgTimer.reset();
                    }
                } else {
                    autoDrive = false;
                    wgFirst = true;
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }


//            GAMEPAD 2
//            if (gamepad2.a || gamepad2.b || gamepad2.y) {
//                if (turnFirst) {
//                    if (gamepad2.a)
//                        turn = new TurnToAngle(-25);
//                    if (gamepad2.b)
//                        turn = new TurnToAngle(-29);
//                    if (gamepad2.y)
//                        turn = new TurnToAngle(-32);
//                    turn.start();
//                    turnFirst = false;
//                }
//
//                if (turn.isFinished()) {
//                    turn.done();
//                } else {
//                    turn.update();
//                }
//            } else {
//                turnFirst = true;
//            }

            if(driverStation.resetRobotPose()) // IDK if this works or not
                Robot.getMecanumDrive().setCurrentPose(new Pose2d(0, 0, 0));

            while (loopTimer.milliseconds() < 20) {
                idle();
            }

            if (on) {
                if (!psOn)
                    Robot.getFlyWheel().setRPM(rpm / 2);
                else
                    Robot.getFlyWheel().setRPM((rpm / 2) - 50);
            }
            else if (!on) {
                Robot.getFlyWheel().turnOffFlyWheel();
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());

//            telemetry.addData("current", Robot.revHubMaster.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

//            -0.4250891276667633
//            Math.abs(-.5)

//            double distance = y / x, height = (Robot.getCamera().getPosOfTarget().y + .5) * 152.04169725643117073448798327336;
//            telemetry.addData("Distance", distance);
//            telemetry.addData("AX", (Math.atan(distance) * 57.2934) - ay);
//            telemetry.addData("Target Height", height);
//            ay = Math.atan(height / 103) * 57.2934;
//            telemetry.addData("ay", ay);
//            telemetry.addData("Dis to Goal", 0.5735 * height + 71.888);

            double camAngle = Robot.getCamera().camYPose;
            telemetry.addData("Dist to Goal", (0.051 * Math.pow(Robot.getCamera().scanForGoal.getPointInPixels().y, 2))
                    - (3.0635 * Robot.getCamera().scanForGoal.getPointInPixels().y) + 117.19);

//            telemetry.addData("angle of cam", Robot.getCamera().camYPose);
            telemetry.addData("pixels", Robot.getCamera().scanForGoal.getPointInPixels());

//            telemetry.addData("Fly Wheel Speed", (Robot.flyWheel.getVelocity() / 28) * 60);
            telemetry.addData("Odometry Adjusted : ", Robot.getMecanumDrive().getAdjustedPose());

            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
//
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setFill("black")
                    .setStrokeWidth(2)
                    .fillRect(Robot.getMecanumDrive().getAdjustedPose().getX() - 72, Robot.getMecanumDrive().getAdjustedPose().getY() - 72, 17, 17.5);
            dashboard.sendTelemetryPacket(packet);

            Robot.getMecanumDrive().setFieldCentricPower(Robot.getMecanumDrive().xSpeed,
                    Robot.getMecanumDrive().ySpeed, Robot.getMecanumDrive().turnSpeed);
            Robot.update();

            // dt timer
            loopTimer.reset();
        }
    }
}
