/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.IMPORTANT_CODE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;
@Disabled
@Autonomous(name = "OpenCVBlueSimple")
//match three games: 205, 202, 209 yeeee vidur pop off, better cycle time than videet, second year replaces third year1`
public class OpenCVBlueSimple extends LinearOpMode {
    OpenCvCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotorEx shooter = null;
    Servo pusher = null;
    Servo wobble_servo2 = null;
    DcMotor intake = null;
    WebcamName webcamName;

    int shooterPower = 1425;
    int lowPower = 1420;
    double firstWobbleDownTime = 2.2;
    double secondWobbleDownTime = 0.9;
    double thirdWobbleDownTime = 1.7;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50 , 0, 3, 13);

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        webcamName = hardwareMap.get(WebcamName.class,"webcam");
        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
        wobble_servo2 = hardwareMap.servo.get("wobble2");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pusher = hardwareMap.servo.get("mover");
        intake = hardwareMap.dcMotor.get("intake");

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName,cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        wobble_servo.setPosition(1); wobble_servo2.setPosition(1);
        wobble_servo2.setPosition(1);
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //   phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        Pose2d startPose = new Pose2d();

        //Middle trajectories
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(40, 5), Math.toRadians(0))
                .splineTo(new Vector2d(85, -11), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1); wobble_servo2.setPosition(1); wobble_servo2.setPosition(1);
                    wobbler_motor.setPower(0.5);
                })
                .addTemporalMarker(firstWobbleDownTime, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0); wobble_servo2.setPosition(0);

                    pusher.setPosition(0.59);

                })

                .build();


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())

                .lineToLinearHeading(new Pose2d(64, -12, Math.toRadians(180)))
                .addTemporalMarker(0.1, () -> {

                    wobbler_motor.setPower(-0.8);
                    shooter.setVelocity(shooterPower);
                })
                .addTemporalMarker(secondWobbleDownTime, () -> {
                    wobbler_motor.setPower(0);


                })

                .build();
//        Trajectory strafer = drive.trajectoryBuilder(traj2.end())
//
//                .strafeLeft(3)
//
//
//
//                .build();

        Trajectory side1 = drive.trajectoryBuilder(traj2.end())
                .lineToLinearHeading(new Pose2d(62, -12, Math.toRadians(180)))
                .build();
        Trajectory forward1 = drive.trajectoryBuilder(side1.end())
                .forward(40)

                .addTemporalMarker(0.99, 0, () -> {
                    //   intake.setPower(0.6);
                })

                .build();

        Trajectory toShoot1 = drive.trajectoryBuilder(forward1.end())
                .lineToLinearHeading(new Pose2d(63, -12, Math.toRadians(180)))

                .build();

//
        Trajectory traj6 = drive.trajectoryBuilder(toShoot1.end())
                .back(5)
                .build();


//      0 trajectories
        Trajectory traj01 = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Vector2d(62, 18), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1); wobble_servo2.setPosition(1);
                    wobbler_motor.setPower(0.5);
                })
                .addTemporalMarker(firstWobbleDownTime, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0); wobble_servo2.setPosition(0);

                    pusher.setPosition(0.59);

                })

                .build();
        Trajectory backwards = drive.trajectoryBuilder(traj01.end())
                .forward(15)
                .build();

        Trajectory traj02 = drive.trajectoryBuilder(traj01.end())
//used to be 61, -10
                .lineToLinearHeading(new Pose2d(62, -15, Math.toRadians(180)))
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);
                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(thirdWobbleDownTime, () -> {
                    wobbler_motor.setPower(0);


                })

                .build();

        Trajectory traj06 = drive.trajectoryBuilder(traj02.end())
                .lineTo(new Vector2d(75, -12))
                .build();


        //Four rings

        Trajectory traj11 = drive.trajectoryBuilder(new Pose2d())

                .splineTo(new Vector2d(105, 18), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1); wobble_servo2.setPosition(1);
                    wobbler_motor.setPower(0.5);

                })
                .addTemporalMarker(firstWobbleDownTime, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0); wobble_servo2.setPosition(0);

                    pusher.setPosition(0.59);

                })

                .build();


        Trajectory traj12 = drive.trajectoryBuilder(traj11.end())
                .lineToLinearHeading(new Pose2d(63, -7, Math.toRadians(180)))
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);
                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(thirdWobbleDownTime, () -> {
                    wobbler_motor.setPower(0);


                })

                .build();

        Trajectory side = drive.trajectoryBuilder(traj12.end())
                .lineToLinearHeading(new Pose2d(62, -12, Math.toRadians(180)))
                .build();
        Trajectory forward = drive.trajectoryBuilder(side.end())
                .forward(40)

                .addTemporalMarker(0.1, () -> {
                    intake.setPower(-1);
                })
                .addTemporalMarker(0.99, 0, () -> {
                    //          intake.setPower(1);
                })

                .build();

        Trajectory toShoot = drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(62, -7, Math.toRadians(180)))
                .addTemporalMarker(0.99, 0, () -> {
                    intake.setPower(0);

                })
                .build();


        Trajectory toPark = drive.trajectoryBuilder(toShoot.end())
                .back(10)
                .build();


        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                drive.followTrajectory(traj1);
                //   sleep(200);

                drive.followTrajectory(traj2);
                //  delay(100);
                //    drive.followTrajectory(strafer);
                shooter.setVelocity(shooterPower);
                //    sleep(1000);
                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                sleep(300);
                shooter.setVelocity(shooterPower);
                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                sleep(300);
                shooter.setVelocity(shooterPower);
                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                //   sleep(200);

                shooter.setPower(0);

                intake.setPower(-0.7);

                drive.followTrajectory(side1);
                //   sleep(200);
                drive.followTrajectory(forward1);
                shooter.setVelocity(shooterPower);
                drive.followTrajectory(toShoot1);
                intake.setPower(0);

                pusher.setPosition(0.45);
                sleep(400);
                pusher.setPosition(0.59);
                shooter.setVelocity(0);
                //   sleep(100);

                drive.followTrajectory(traj6);
                PoseStorage.currentPose = drive.getPoseEstimate();
                sleep(100);
                sleep(200);
                PoseStorage.currentPose = drive.getPoseEstimate();
            } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                drive.followTrajectory(traj11);
                //  sleep(200);

                drive.followTrajectory(traj12);

                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                sleep(300);
                shooter.setVelocity(shooterPower);
                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);

                shooter.setVelocity(shooterPower);
                sleep(300);
                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                sleep(200);


                shooter.setPower(0);

                //  intake.setPower(-0.8);

                drive.followTrajectory(side);
                //     sleep(200);
                drive.followTrajectory(forward);
                //       sleep(100);
                //    drive.turn(Math.toRadians(-52));

                shooter.setVelocity(shooterPower);
                drive.followTrajectory(toShoot);
                intake.setPower(0);

                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                sleep(300);

                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
//
                //   shooter.setVelocity(2450*0.99);
                sleep(300);
                pusher.setPosition(0.45);
                sleep(300);
                pusher.setPosition(0.59);
                sleep(200);

                intake.setPower(0);
                shooter.setPower(0);
                drive.followTrajectory(toPark);
                PoseStorage.currentPose = drive.getPoseEstimate();
                sleep(200);

            } else {
                drive.followTrajectory(traj01);
                sleep(200);
                drive.followTrajectory(backwards);
                sleep(200);

                drive.followTrajectory(traj02);
                shooter.setVelocity(shooterPower);
                sleep(1000);
                pusher.setPosition(0.45);
                sleep(400);
                pusher.setPosition(0.59);
                sleep(400);
                shooter.setVelocity(shooterPower);
                pusher.setPosition(0.45);
                sleep(400);
                pusher.setPosition(0.59);
                sleep(1000);
                shooter.setVelocity(shooterPower);
                pusher.setPosition(0.45);
                sleep(400);
                pusher.setPosition(0.59);
                sleep(200);

                shooter.setPower(0);
                PoseStorage.currentPose = drive.getPoseEstimate();
                sleep(500);
                wobble_servo.setPosition(0); wobble_servo2.setPosition(0);
                sleep(200);
                drive.followTrajectory(traj06);
            }
            break;
        }
    }


    public void setDirectionForward(boolean value, double time, double power) { //time should be in milliseconds
        String frontRightMotorName = "front_right";
        String backrightMotorName = "back_right";
        String frontleftMotorName = "front_left";
        String backleftMotorName = "back_left";


        DcMotor backRight = hardwareMap.dcMotor.get(backrightMotorName);
        DcMotor backLeft = hardwareMap.dcMotor.get(backleftMotorName);
        DcMotor frontRight = hardwareMap.dcMotor.get(frontRightMotorName);
        DcMotor frontLeft = hardwareMap.dcMotor.get(frontleftMotorName);

        double startTime = System.currentTimeMillis();
        if (value) {//forward
            telemetry.addData("forward", "starting");
            telemetry.update();
            backRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setPower(-power);
            backLeft.setPower(-power);
            frontLeft.setPower(-power);
            frontRight.setPower(-power);


            while (opModeIsActive()) {
                if ((System.currentTimeMillis() - startTime) >= time) {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    telemetry.addData("stopped", "moving on");
                    telemetry.update();
                    break;
                }
            }

        } else {//backward
            backRight.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backRight.setPower(-power);
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(power);

            while (opModeIsActive()) {
                if ((System.currentTimeMillis() - startTime) >= time) {
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    break;
                }
            }
        }
    }

    public void delay(double delayTime) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < delayTime && opModeIsActive()) ;
        return;
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(270,192);

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 157;
        final int ONE_RING_THRESHOLD = 130;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }
}