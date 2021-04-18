package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.flywheel.PowerShotsAuto;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.auto.actions.teleop.PowerShots;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.ftc2020.subsystems.FlyWheel;
import com.team9889.ftc2020.subsystems.Robot;
import com.team9889.lib.control.controllers.PID;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class Teleop extends Team9889Linear {
    public static double x = 103, y = 42, ay = 19.58, ax = 2.603092035700193, multiplier = 1.03;
    public static int timeToWait = 150;

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

    public static double rpm = 1240, rpm12 = 1350, rpm11 = 1370, rpm10 = 1380;

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

        wgTimer.reset();
        Robot.flyWheel.resetEncoder();

        while (opModeIsActive()) {
            if (!shooting) {
                if (Robot.result > 12) {
                    rpm = rpm12;
                } else if (Robot.result <= 12 && Robot.result > 11.5) {
                    rpm = rpm11;
                } else if (Robot.result <= 11.5) {
                    rpm = rpm10;
                }
            }


            if (Robot.getCamera().getPosOfTarget().x != 1e10) {
                dist = 37.852 * Math.exp(0.0192 * Robot.getCamera().scanForGoal.getPointInPixels().y);
            }

            if (gamepad2.right_trigger > .1 && !shooting) {
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

                if (Math.abs(Robot.getCamera().getPosOfTarget().x) < .1) {
                    if (readyCount >= 3) {
                        Robot.wgGrabber.setPosition(.75);
                    } else {
                        Robot.wgGrabber.setPosition(.25);
                    }
                    readyCount++;
                } else {
                    readyCount = 0;
                }

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

                if (driverStation.getStartIntaking()) {
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

                if (gamepad1.x) {
                    Robot.getIntake().SetFrontIntakePower(0);
                    Robot.backIntake.setPower(1);
                    Robot.passThrough.setPower(1);
                }

                if (driverStation.getFW() && (!Robot.getFlyWheel().psPower)){
                    on = true;
                    Robot.getFlyWheel().psPower = false;
                } else if (!driverStation.getFW() && (!Robot.getFlyWheel().psPower || gamepad1.x)) {
                    on = false;
                    Robot.getFlyWheel().psPower = false;
                }

                if (Robot.getFlyWheel().psPower) {
                    timeToWait = 200;
                }

                if (Math.abs(Robot.getCamera().getPosOfTarget().x) < 0.1) {
                    ready++;
                } else if (!shooting) {
                    ready = 0;
                }

                if (!driveToPos) {
                    if (false) {
                        if (armTimer.milliseconds() > timeToWait) {
                            if (extend) {
                                Robot.fwArm.setPosition(.5);
                                extend = false;
                            } else {
                                Robot.fwArm.setPosition(.65);
                                ringsPassed = 0;
                                extend = true;
                            }

                            armTimer.reset();
                        }
                    } else if ((gamepad1.right_bumper || (gamepad2.right_trigger > .1 &&
                            ready > 5)) && on) {
                        Robot.fwLock.setPosition(.4);
                        if (armTimer.milliseconds() > timeToWait) {
                            if (extend) {
                                Robot.fwArm.setPosition(0.5);
                                extend = false;
                            } else {
                                Robot.fwArm.setPosition(.65);
                                setRpm = rpm;
                                ringsPassed = 0;
                                extend = true;
                            }

                            armTimer.reset();
                            shooting = true;
                        }
                    } else if (armTimer.milliseconds() > timeToWait && !Robot.getFlyWheel().psPower) {
                        Robot.fwLock.setPosition(1);
                        Robot.fwArm.setPosition(0.5);
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

                if (gamepad2.left_bumper) {
                    Robot.wgLeft.setPosition(.3);
                    Robot.wgRight.setPosition(.3);
                    Robot.wgGrabber.setPosition(.52);
                } else {
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
                        if (wgTimer.milliseconds() < 500) {
                            Robot.wgGrabber.setPosition(0.25);
                        } else {
                            Robot.wgLeft.setPosition(.4);
                            Robot.wgRight.setPosition(.4);
                        }

                        lastWGState = false;
                    } else if (driverStation.getWG()) {
                        if (wgTimer.milliseconds() < 500) {
                            Robot.wgLeft.setPosition(0.9);
                            Robot.wgRight.setPosition(0.9);
                        } else {
                            Robot.wgGrabber.setPosition(.52);
                            Robot.wgLeft.setPosition(0.9);
                            Robot.wgRight.setPosition(0.9);
                        }

                        lastWGState = true;
                    }
                }

                if (gamepad2.right_bumper){
                    Robot.wgGrabber.setPosition(.52);
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
                } else {
                    autoDrive = false;
                    wgFirst = true;
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
            }

            if (Robot.ringDetector.getDistance(DistanceUnit.INCH) < 2.7 ) {
                if (!ringFront) {
                    ringFront = true;
                } else if (ringHalf) {
                    ringsPassed++;
                }

                ringTimer.reset();
            } else if (ringFront) {
//                if (ringTimer.milliseconds() > 800) {
//                    ringFront = false;
//                    ringHalf = false;
//                    ringsPassed++;
//                }

                ringHalf = true;
            }

            if (gamepad2.left_stick_button && gamepad2.right_stick_button && !resetPressed) {
                Robot.getMecanumDrive().setCurrentPose(new Pose2d(0, 0, 0));
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
                if (!Robot.getFlyWheel().psPower) {
                    Robot.getFlyWheel().setRPM(rpm);
                }
            }
            else if (!on && !Robot.getFlyWheel().psPower) {
                Robot.getFlyWheel().setMode(FlyWheel.Mode.OFF);
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());
            telemetry.addData("Power Shot Power", Robot.getFlyWheel().psPower);
            telemetry.addData("Distance", dist + "");
            telemetry.addData("Rings", ringsPassed);

            telemetry.addData("Volts", Robot.result);
            telemetry.addData("Current RPM", setRpm);

            telemetry.addData("pixels", Robot.getCamera().scanForGoal.getPointInPixels());

            telemetry.addData("Fly Wheel Speed", (Robot.flyWheel.getVelocity() / 28) * 60);
            telemetry.addData("Odometry Adjusted : ", Robot.getMecanumDrive().getAdjustedPose());

            telemetry.addData("sensor", Robot.ringDetector.getDistance(DistanceUnit.INCH));

            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
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

            while (loopTimer.milliseconds() < 20) {

            }
            loopTimer.reset();
        }
    }
}
