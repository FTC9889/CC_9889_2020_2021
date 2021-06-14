package com.team9889.ftc2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2020.auto.actions.Action;
import com.team9889.ftc2020.auto.actions.flywheel.PowerShotsTeleOp;
import com.team9889.ftc2020.auto.actions.teleop.AimAndShoot;
import com.team9889.ftc2020.subsystems.Camera;
import com.team9889.ftc2020.subsystems.FlyWheel;

import java.util.ArrayList;

/**
 * Created by MannoMation on 1/14/2019.
 */

@TeleOp
@Config
public class Teleop extends Team9889Linear {
    public static double x = 103, y = 42, ay = 19.58, ax = 2.603092035700193, multiplier = 3;
    public static int timeToWait = 80;

    boolean wgUse = false;

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

    public static double wantedRpm = 1500, rampAngle = .4, rpm = 1500, trueRPM = 2900, lowRPMTolerance = 350, hightRPMTolerance = 100;

    boolean wgInPos = false;
    boolean wgFirst = true;

    int ringsPassed = 0;

    ArrayList<Action> actions = new ArrayList<>();

    boolean middleGoal = false;

    @Override
    public void runOpMode() {
        DriverStation driverStation = new DriverStation(gamepad1, gamepad2);
        waitForStart(false);

        Robot.fwFlap.setPosition(.5);

        wgTimer.reset();
        Robot.flyWheel.resetEncoder();

        Action psAction = new PowerShotsTeleOp();
        psAction.start();

        Robot.leftArm.setPosition(0);
        Robot.rightArm.setPosition(1);

        while (opModeIsActive()) {
            if (gamepad2.left_stick_button) {
                Robot.blue = true;
            } else if (gamepad2.right_stick_button) {
                Robot.blue = false;
            }

            if (gamepad1.dpad_right) {
                if (psFirst) {
                    ps = new PowerShotsTeleOp();
                    ps.start();

                    psFirst = false;
                }
                if (!ps.isFinished()) {
                    Robot.getFlyWheel().psPower = true;
                    ps.update();
                } else {
                    ps.done();
                }
            } else if (gamepad2.a) {
                if (psFirst) {
                    ps = new PowerShotsTeleOp(true, false, false);
                    ps.start();

                    psFirst = false;
                }
                if (!ps.isFinished()) {
                    Robot.getFlyWheel().psPower = true;
                    ps.update();
                } else {
                    ps.done();
                }
            } else if (gamepad2.b) {
                if (psFirst) {
                    ps = new PowerShotsTeleOp(false, true, false);
                    ps.start();

                    psFirst = false;
                }
                if (!ps.isFinished()) {
                    Robot.getFlyWheel().psPower = true;
                    ps.update();
                } else {
                    ps.done();
                }
            } else if (gamepad2.y) {
                if (psFirst) {
                    ps = new PowerShotsTeleOp(false, false, true);
                    ps.start();

                    psFirst = false;
                }
                if (!ps.isFinished()) {
                    Robot.getFlyWheel().psPower = true;
                    ps.update();
                } else {
                    ps.done();
                }
            } else if (!psFirst){
                Robot.getFlyWheel().psPower = false;
                ps.done();
                psFirst = true;
//                    Robot.flyWheel.setPower(rpm);
            }

            x = Math.abs(Robot.getCamera().getPosOfTarget().y);

            if (!shooting && !middleGoal) {
                if (Robot.result > 12.5) {
                    rpm = wantedRpm - 30;
                } else if (Robot.result <= 12.5 && Robot.result > 12) {
                    rpm = wantedRpm - 15;
                } else if (Robot.result <= 11) {
                    rpm = wantedRpm;
                }
                trueRPM = 2900;
            } else if (!shooting) {
                rpm = 1300;
                trueRPM = 2700;
            }

            Robot.fwFlap.setPosition(rampAngle);

            if (gamepad2.left_bumper && !shooting) {
                if (Robot.getCamera().currentCamState != Camera.CameraStates.GOAL) {
                    Robot.getCamera().camYPose = .7;
                    Robot.getCamera().setGoalCamPos();
                    camTimer.reset();
                }

                Robot.getCamera().setGoalCamPos();

                Robot.getCamera().setScanForGoal();

                if (camTimer.milliseconds() > 700) {

                    if (driveFirst) {
                        drive = new AimAndShoot();
                        drive.start();

                        driveFirst = false;
                    }
                    drive.update();
                }
            }

            // If not resetting imu, normal operation
            if(!driverStation.resetIMU()) {
                // Drive
                Robot.getMecanumDrive().xSpeed += driverStation.getX();
                Robot.getMecanumDrive().ySpeed += driverStation.getY();
                Robot.getMecanumDrive().turnSpeed += driverStation.getSteer();

                if (driverStation.getStartIntaking()) {
                    Robot.getIntake().SetFrontIntakePower(1);
                    Robot.backIntake.setPower(0);
                    Robot.getIntake().SetPassThroughPower(1);
                } else if (driverStation.getStopIntaking()) {
                    Robot.getIntake().SetFrontIntakePower(0);
                    Robot.backIntake.setPower(0);
                    Robot.getIntake().SetPassThroughPower(0);
                } else if (driverStation.getStartOuttaking()) {
                    Robot.getIntake().SetFrontIntakePower(-1);
                    Robot.backIntake.setPower(-1);
                    Robot.getIntake().SetPassThroughPower(-1);
                }
                if (gamepad2.left_bumper && Robot.frontIntake.motor.getPower() == 0) {
//                   Robot.getIntake().SetFrontIntakePower(0);
//                   Robot.backIntake.setPower(0);
                    Robot.getIntake().SetPassThroughPower(0.4);
                }

                if (gamepad1.x) {
                    Robot.getIntake().SetFrontIntakePower(0);
                    Robot.backIntake.setPower(1);
                    Robot.getIntake().SetPassThroughPower(1);
                }

                if (driverStation.getFW() && (!Robot.getFlyWheel().psPower) && !driverStation.getPS()){
                    on = true;
                    Robot.getFlyWheel().psPower = false;
                    timeToWait = 80;
                } else if (!driverStation.getFW() && (!Robot.getFlyWheel().psPower || gamepad1.x)) {
                    on = false;
                    Robot.getFlyWheel().psPower = false;
                }

//                if (Robot.getFlyWheel().psPower) {
//                    timeToWait = 200;
//                }

                if (Math.abs(Math.round(Robot.getCamera().getPosOfTarget().x * 100.0))/100.0 < 0.15) {
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
//                    } else if (gamepad1.right_bumper || gamepad2.y) {
                        Robot.getIntake().ringsIntaken = 0;

                        Robot.fwLock.setPosition(.5);
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
                    } else if (armTimer.milliseconds() > timeToWait && !Robot.getFlyWheel().psPower) {
                        Robot.fwLock.setPosition(1);
                        Robot.fwArm.setPosition(0.47);
                        extend = true;
                        shooting = false;
                    }
                }

                if (gamepad1.left_bumper) {
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
                            Robot.wgLeft.setPosition(.44);
                            Robot.wgRight.setPosition(.44);
                        }
                    } else {
                        if (wgTimer.milliseconds() < 500) {
//                            Robot.wgGrabber.setPosition(0.25);
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
                        Robot.wgGrabber.setPosition(.7);
                        Robot.wgLeft.setPosition(0.8);
                        Robot.wgRight.setPosition(0.8);
                    }

                    wgUse = true;

                    lastWGState = true;
                }
//                }

                if (gamepad2.right_bumper){
                    Robot.wgGrabber.setPosition(.7);
                }

                if (driverStation.getPS()) {
                    middleGoal = true;
                    timeToWait = 300;
                    Robot.autoWG.setPosition(0.5);
                    on = true;
                } else if (driverStation.getGoal()) {
                    middleGoal = true;
                    Robot.blueGoal = !Robot.blue;
                    Robot.autoWG.setPosition(0.9);
                } else {
                    middleGoal = false;
                    Robot.blueGoal = Robot.blue;
                    Robot.autoWG.setPosition(0.1);
                }

            } else {
                Robot.getMecanumDrive().setPower(0,0,0);
                Robot.getMecanumDrive().writeAngleToFile();
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
                Robot.getFlyWheel().power = 0;
                Robot.getFlyWheel().counter = 0;
                Robot.getFlyWheel().setMode(FlyWheel.Mode.OFF);
            }

            telemetry.addData("Loop Time", loopTimer.milliseconds());
            telemetry.addData("Blue", Robot.blue);
            telemetry.addData("Volts", Robot.result);

            Robot.outputToTelemetry(telemetry);
            telemetry.update();

            FtcDashboard.getInstance().startCameraStream(Robot.camera, 0);
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);

            Robot.getMecanumDrive().setFieldCentricPower(Robot.getMecanumDrive().xSpeed,
                    Robot.getMecanumDrive().ySpeed, Robot.getMecanumDrive().turnSpeed, Robot.blue);
            Robot.update();

            loopTimer.reset();
        }
    }
}
