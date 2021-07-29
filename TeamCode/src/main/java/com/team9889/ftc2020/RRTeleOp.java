package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.auto.actions.teleop.PowerShots;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.lib.control.controllers.PID;

import java.util.ArrayList;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class RRTeleOp extends Team9889Linear {
    public static double x = 103, y = 42, ay = 19.58, ax = 2.603092035700193, multiplier = 3;
    public static int timeToWait = 80;

    boolean wgUse = false;

    boolean blue = false;

    double setRpm = 0;

    private ElapsedTime loopTimer = new ElapsedTime(), readyTimer = new ElapsedTime();
    ElapsedTime armTimer = new ElapsedTime(), wgTimer = new ElapsedTime(), wgAutoTimer = new ElapsedTime();
    boolean on = false;

    double ready = 0;
    boolean extend = false;

    boolean driveToPos = false, autoDrive = false, shooting = false;
    boolean driveFirst = true, turnFirst = true, psFirst = true;
    Action drive, turn, ps;

    boolean lastWGState = false;

    ElapsedTime camTimer = new ElapsedTime();

    boolean resetPressed = false;

    private PID orientationPID = new PID(1, 0, 100);


    public static double wantedRpm = 1520, rampAngle = .4, rpm = 1520, trueRPM = 2900, lowRPMTolerance = 350, hightRPMTolerance = 100;

    boolean wgInPos = false;
    boolean wgFirst = true;
    boolean autoAimreleased = false;
    int readyCount = 0;

    double dist = 0;

    boolean ringFront = false, ringHalf = false;
    int ringsPassed = 0;
    ElapsedTime ringTimer = new ElapsedTime();

    ArrayList<Action> actions = new ArrayList<>();

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        Robot.fwFlap.setPosition(.5);

        wgTimer.reset();
        Robot.flyWheel.resetEncoder();

        while (opModeIsActive()) {
            if (gamepad2.a) {
                blue = true;
            } else if (gamepad2.b) {
                blue = false;
            }

            x = Math.abs(Robot.getCamera().getPosOfTarget().y);
//            wantedRpm = (1E+06 * Math.pow(x, 6)) - (2E+06 * Math.pow(x, 5)) + (2E+06 * Math.pow(x, 4)) -
//                    (668141 * Math.pow(x, 3)) + (122827 * Math.pow(x, 2)) - (10904 * (x)) + 1941.6;
//            if (x != 1e10)
//                wantedRpm = 4112.2 * Math.pow(x, 3) - 2586.5 * Math.pow(x, 2) + 74.118 * x + 1594;

//            if (wantedRpm > 1600) {
//                wantedRpm = 1600;
//            }
            telemetry.addData("Wanted RPM", wantedRpm);

            if (!shooting) {
                if (Robot.result > 12.5) {
                    rpm = wantedRpm - 30;
                } else if (Robot.result <= 12.5 && Robot.result > 12) {
                    rpm = wantedRpm - 15;
                } else if (Robot.result <= 11) {
                    rpm = wantedRpm;
                }
            }

            Robot.fwFlap.setPosition(rampAngle);

            if (gamepad2.left_bumper && !shooting) {
//                rpm = ((3.06 * dist) + 1041) * multiplier;

                if (Robot.getCamera().currentCamState != Camera.CameraStates.GOAL) {
                    Robot.getCamera().camYPose = .7;
                    Robot.getCamera().setGoalCamPos();
                    camTimer.reset();
                }

//                if (gamepad2.a) {
//                    Robot.getCamera().setPS1CamPos();
//                    rpm = 2340;
//                } else if (gamepad2.b) {
//                    Robot.getCamera().setPS2CamPos();
//                    rpm = 2340;
//                } else if (gamepad2.y){
//                    Robot.getCamera().setPS3CamPos();
//                    rpm = 2340;
//                } else
//                if (gamepad2.right_trigger > .1) {
                Robot.getCamera().setGoalCamPos();
//                }

                Robot.getCamera().setScanForGoal();

                if (camTimer.milliseconds() > 700) {

                    if (driveFirst) {
                        drive = new AimAndShoot();
                        drive.start();

                        driveFirst = false;
                    }
                    drive.update();
                }

//                if (Math.abs(Robot.getCamera().getPosOfTarget().x) < .1) {
//                    if (readyCount >= 3) {
//                        Robot.wgGrabber.setPosition(.75);
//                    } else {
//                        Robot.wgGrabber.setPosition(.25);
//                    }
//                    readyCount++;
//                } else {
//                    readyCount = 0;
//                }

                autoAimreleased = true;
            } else if (autoAimreleased) {
                Robot.wgGrabber.setPosition(.25);
                autoAimreleased = false;
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
                // Drive
                Robot.getMecanumDrive().xSpeed += driverStation.getX();
                Robot.getMecanumDrive().ySpeed += driverStation.getY();
                Robot.getMecanumDrive().turnSpeed += driverStation.getSteer();

                if (gamepad1.dpad_right) {
                    if (psFirst) {
                        ps = new PowerShots();
                        ps.start();

                        psFirst = false;
                    }
                    if (!ps.isFinished()) {
                        ps.update();
                    } else {
                        ps.done();
                    }
                } else if (!psFirst){
                    ps.done();
                    psFirst = true;
//                    Robot.flyWheel.setPower(rpm);
                }

                if (driverStation.getStartFrontIntaking()) {
                    Robot.getIntake().SetFrontIntakePower(1);
                    Robot.backIntake.setPower(0);
                    Robot.passThrough.setPower(1);
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().SetFrontIntakePower(0);
                    Robot.backIntake.setPower(0);
                    Robot.passThrough.setPower(0);
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().SetFrontIntakePower(-1);
                    Robot.backIntake.setPower(-1);
                    Robot.passThrough.setPower(-1);
                }
                if (gamepad2.left_bumper && Robot.frontIntake.motor.getPower() == 0) {
//                   Robot.getIntake().SetFrontIntakePower(0);
//                   Robot.backIntake.setPower(0);
                    Robot.passThrough.setPower(.4);
                }

                if (gamepad1.x) {
                    Robot.getIntake().SetFrontIntakePower(0);
                    Robot.backIntake.setPower(1);
                    Robot.passThrough.setPower(1);
                }

                if (driverStation.getFW() && (!Robot.getFlyWheel().autoPower)){
                    on = true;
                    Robot.getFlyWheel().autoPower = false;
                } else if (!driverStation.getFW() && (!Robot.getFlyWheel().autoPower || gamepad1.x)) {
                    on = false;
                    Robot.getFlyWheel().autoPower = false;
                }

//                if (Robot.getFlyWheel().psPower) {
//                    timeToWait = 200;
//                }

                if (Math.abs(Robot.getCamera().getPosOfTarget().x) < 0.15) {
                    ready++;
                } else if (!shooting) {
                    ready = 0;
                }

                if (!driveToPos) {
                    if (false) {
                        if (armTimer.milliseconds() > timeToWait) {
                            if (extend) {
                                Robot.fwArm.setPosition(.47);
                                extend = false;
                            } else {
                                Robot.fwArm.setPosition(.62);
                                ringsPassed = 0;
                                extend = true;
                            }

                            armTimer.reset();
                        }
                    } else if (gamepad1.right_bumper || (gamepad2.left_bumper &&
                            ready > 5) && on ) {
                        Robot.fwLock.setPosition(.4);
                        if ((-lowRPMTolerance < (Robot.getFlyWheel().getRPM() - trueRPM) &&
                                (Robot.getFlyWheel().getRPM() - trueRPM) < hightRPMTolerance)) {
                            if (armTimer.milliseconds() > timeToWait) {
                                if (extend) {
                                    Robot.fwArm.setPosition(0.47);
                                    extend = false;
                                } else {
                                    Robot.fwArm.setPosition(.62);
                                    setRpm = rpm;
                                    ringsPassed = 0;
                                    extend = true;
                                }

                                armTimer.reset();
                                shooting = true;
                            }
                        }
                    } else if (armTimer.milliseconds() > timeToWait && !Robot.getFlyWheel().autoPower) {
                        Robot.fwLock.setPosition(1);
                        Robot.fwArm.setPosition(0.47);
                        extend = true;
                        shooting = false;
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

//                if (gamepad2.left_bumper) {
//                    Robot.wgLeft.setPosition(.3);
//                    Robot.wgRight.setPosition(.3);
//                    Robot.wgGrabber.setPosition(.52);
//                } else {
                if (autoDrive && !wgInPos) {
                    Robot.wgLeft.setPosition(.2);
                    Robot.wgRight.setPosition(.2);
                } else if (wgInPos) {
                    if (wgAutoTimer.milliseconds() < 500) {
                        Robot.wgLeft.setPosition(.4);
                        Robot.wgRight.setPosition(.4);
                    } else if (wgAutoTimer.milliseconds() > 500 && wgAutoTimer.milliseconds() < 1000) {
                        Robot.wgGrabber.setPosition(0.25);
                        Robot.wgLeft.setPosition(.4);
                        Robot.wgRight.setPosition(.4);
                    } else {
                        Robot.wgLeft.setPosition(.8);
                        Robot.wgRight.setPosition(.8);
                    }
                } else if (!driverStation.getWG()) {
                    if (wgUse) {
                        if (wgTimer.milliseconds() < 500) {
                            Robot.wgGrabber.setPosition(0.25);
                        } else {
                            Robot.wgLeft.setPosition(.5);
                            Robot.wgRight.setPosition(.5);
                        }
                    } else {
                        if (wgTimer.milliseconds() < 500) {
                            Robot.wgGrabber.setPosition(0.25);
                        } else {
                            Robot.wgLeft.setPosition(.4);
                            Robot.wgRight.setPosition(.4);
                        }
                    }

                    lastWGState = false;
                } else if (driverStation.getWG()) {
                    if (wgTimer.milliseconds() < 500) {
                        Robot.wgLeft.setPosition(0.8);
                        Robot.wgRight.setPosition(0.8);
                    } else {
                        Robot.wgGrabber.setPosition(.52);
                        Robot.wgLeft.setPosition(0.8);
                        Robot.wgRight.setPosition(0.8);
                    }

                    wgUse = true;

                    lastWGState = true;
                }
//                }

                if (gamepad2.right_bumper){
                    Robot.wgGrabber.setPosition(.52);
                }

                if (gamepad1.left_trigger > .5 && !wgInPos) {
                    if (wgFirst) {
//                        Robot.getCamera().setWGCamPos();
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
                } else {
                    autoDrive = false;
                    wgFirst = true;
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }

            if (gamepad2.left_stick_button && gamepad2.right_stick_button && !resetPressed) {
//                Robot.getMecanumDrive().setCurrentPose(new Pose2d(0, 0, 0));
                resetPressed = true;
            } else if (!gamepad2.left_stick_button && !gamepad2.right_stick_button) {
                resetPressed = false;
            }

            if (gamepad2.dpad_down) {
                Robot.leftArm.setPosition(1);
                Robot.rightArm.setPosition(0);
            } else if (gamepad2.dpad_left) {
                Robot.leftArm.setPosition(0);
                Robot.rightArm.setPosition(1);
            }

            if (on) {
                if (!Robot.getFlyWheel().autoPower) {
                    Robot.getFlyWheel().setRPM(rpm);
                }
            }
            else if (!on && !Robot.getFlyWheel().autoPower) {
                Robot.getFlyWheel().power = 0;
                Robot.getFlyWheel().counter = 0;
//                Robot.getFlyWheel().setMode(FlyWheel.Mode.OFF);
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());
            telemetry.addData("Blue", blue);

//            telemetry.addData("Right Dist", Robot.rightDist.getDistance(DistanceUnit.INCH));
//            telemetry.addData("Left Dist", Robot.leftDist.getDistance(DistanceUnit.INCH));

//            telemetry.addData("Kf", Robot.getFlyWheel().KfEstimator.getAverage());
            telemetry.addData("Power Shot Power", Robot.getFlyWheel().autoPower);
            telemetry.addData("Distance", dist + "");
            telemetry.addData("Rings", ringsPassed);

            telemetry.addData("Volts", Robot.result);
            telemetry.addData("Current RPM", setRpm);

//            telemetry.addData("pixels", Robot.getCamera().scanForGoal.getPointInPixels());

            telemetry.addData("Fly Wheel Speed", Robot.getFlyWheel().getRPM());
//            telemetry.addData("Odometry Adjusted : ", Robot.getMecanumDrive().getAdjustedPose());

            Robot.outputToTelemetry(telemetry);
            telemetry.update();


            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
            TelemetryPacket packet = new TelemetryPacket();

            Canvas fieldOverlay = packet.fieldOverlay();

            // Draw bot on canvas
//            fieldOverlay.setStroke("#3F51B5");
//            packet.fieldOverlay(fieldOverlay, Robot.rr.getPoseEstimate());

            packet.fieldOverlay()
                    .setFill("black")
                    .setStrokeWidth(2)
                    .fillRect(Robot.rr.getPoseEstimate().getX(), Robot.rr.getPoseEstimate().getY(), 17, 17.5);
            dashboard.sendTelemetryPacket(packet);

//            Robot.getMecanumDrive().setFieldCentricPower(Robot.getMecanumDrive().xSpeed,
//                    Robot.getMecanumDrive().ySpeed, Robot.getMecanumDrive().turnSpeed, blue);
            Robot.update();
            Robot.rr.getLocalizer().update();

            // dt timer

//            while (loopTimer.milliseconds() < 20) {
//
//            }
            loopTimer.reset();
        }
    }
}
