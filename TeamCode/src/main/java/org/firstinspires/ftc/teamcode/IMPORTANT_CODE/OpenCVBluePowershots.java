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
@Autonomous(name = "OpenCVBluePowershots")
//match three games: 205, 202, 209 yeeee vidur pop off, better cycle time than videet, second year replaces third year1`

//RED ALTERNATE SHOOTING: -42, 11 at 2420 with 170 heading
public class OpenCVBluePowershots extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 3, 13);
    OpenCvCamera phoneCam;
    WebcamName webcamName;
    SkystoneDeterminationPipeline pipeline;
    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotorEx shooter = null;
    Servo pusher = null;
    DcMotor intake = null;
    Servo wobble_servo2;
    double firstWobble = 0.9;
    double secondWobble = 0;

    @Override
    public void runOpMode() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
        wobble_servo2 = hardwareMap.servo.get("wobble2");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pusher = hardwareMap.servo.get("mover");
        intake = hardwareMap.dcMotor.get("intake");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        wobble_servo.setPosition(1);wobble_servo2.setPosition(1);
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //   phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        int shooterPower = 1330;

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        Pose2d startPose = new Pose2d();
        Trajectory traj01 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, -12, Math.toRadians(0)))


                .build();


        Trajectory traj02 = drive.trajectoryBuilder(traj01.end())

                .lineToLinearHeading(new Pose2d(62, -20, Math.toRadians(180)))
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);

                })

                .build();

        Trajectory middleShot0 = drive.trajectoryBuilder(traj02.end())
                .lineToLinearHeading(new Pose2d(62, -14, Math.toRadians(180)))
                .build();

        Trajectory leftShot0 = drive.trajectoryBuilder(middleShot0.end())
                .lineToLinearHeading(new Pose2d(62, -7, Math.toRadians(180)))
                .build();
//        Trajectory strafer = drive.trajectoryBuilder(traj2.end())
//
//                .strafeLeft(3)
//
//
//
//                .build();
        Trajectory toShoot01 = drive.trajectoryBuilder(leftShot0.end())
                .lineToLinearHeading(new Pose2d(112, -12, Math.toRadians(-90)))


                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })

                .build();
        Trajectory toShoot02 = drive.trajectoryBuilder(toShoot01.end())
                .lineToLinearHeading(new Pose2d(112, 20, Math.toRadians(0)))

                .build();

        Trajectory toWobble = drive.trajectoryBuilder(toShoot02.end())
                .lineToLinearHeading(new Pose2d(90, 20, Math.toRadians(0)))
                .addDisplacementMarker(0.95, 0.5, () -> {


                    wobble_servo.setPosition(0);wobble_servo2.setPosition(0);
                })
                .build();
//

        Trajectory finalTraj0 = drive.trajectoryBuilder(toWobble.end())
                .forward(20)
                .build();
        Trajectory finalTraj01 = drive.trajectoryBuilder(finalTraj0.end())
                .strafeRight(40)
                .build();
        Trajectory finalTraj02 = drive.trajectoryBuilder(finalTraj01.end())
                .back(20)
                .build();

        Trajectory toShoot1 = drive.trajectoryBuilder(leftShot0.end())
                .lineToLinearHeading(new Pose2d(95, -2, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {


                    wobble_servo.setPosition(0);wobble_servo2.setPosition(0);
                })
                .build();


//

        Trajectory finalTraj = drive.trajectoryBuilder(toShoot1.end())
                .forward(10)
                .build();
        Trajectory finalfinal = drive.trajectoryBuilder(finalTraj.end())
                .strafeRight(20)
                .build();


//      0 trajectories


        Trajectory toShoot11 = drive.trajectoryBuilder(leftShot0.end())
                .lineToLinearHeading(new Pose2d(112, -12, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .build();
        Trajectory toShoot12 = drive.trajectoryBuilder(toShoot11.end())
                .lineToLinearHeading(new Pose2d(112, 18, Math.toRadians(-90)))

                .addDisplacementMarker(0.95, 0.5, () -> {


                    wobble_servo.setPosition(0);wobble_servo2.setPosition(0);
                })
                .build();


//

        Trajectory finalTraj1 = drive.trajectoryBuilder(toShoot12.end())
                .forward(40)
                .build();

        Trajectory finalfinal1 = drive.trajectoryBuilder(finalTraj1.end())
                .strafeRight(40)
                .build();
        sleep(1000);
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

            SkystoneDeterminationPipeline.RingPosition initial = pipeline.position;

            // Don't burn CPU cycles busy-looping in this sample

            int sleepTime = 3000;

            sleep(sleepTime);
            drive.followTrajectory(traj01);
            drive.followTrajectory(traj02);

            shooter.setVelocity(shooterPower);
            sleep(1000);
            pusher.setPosition(0.45);
            sleep(400);
            pusher.setPosition(0.58);
            sleep(400);
            drive.followTrajectory(middleShot0);
            shooter.setVelocity(shooterPower-20);
            pusher.setPosition(0.45);
            sleep(400);
            pusher.setPosition(0.58);
            sleep(1000);
            drive.followTrajectory(leftShot0);
            shooter.setVelocity(shooterPower);
            pusher.setPosition(0.45);
            sleep(400);
            pusher.setPosition(0.58);
            sleep(200);

            shooter.setPower(0);



            if (initial == SkystoneDeterminationPipeline.RingPosition.ONE) {

                drive.followTrajectory(toShoot1);
                drive.followTrajectory(finalTraj);
                drive.followTrajectory(finalfinal);
                PoseStorage.currentPose = drive.getPoseEstimate();
                sleep(100);
                wobble_servo.setPosition(0.0);
                sleep(200);
                PoseStorage.currentPose = drive.getPoseEstimate();
            } else if (initial == SkystoneDeterminationPipeline.RingPosition.FOUR) {

                drive.followTrajectory(toShoot11);
                sleep(200);
                drive.followTrajectory(toShoot12);

                drive.followTrajectory(finalTraj1);
                drive.followTrajectory(finalfinal1);
                sleep(1000);
                wobble_servo.setPosition(1);wobble_servo2.setPosition(1);

            } else {


                drive.followTrajectory(toShoot01);
                sleep(200);
                drive.followTrajectory(toShoot02);

                drive.followTrajectory(toWobble);
                drive.followTrajectory(finalTraj0);
                drive.followTrajectory(finalTraj01);
                drive.followTrajectory(finalTraj02);
                sleep(1000);
                //      wobble_servo.setPosition(1);wobble_servo2.setPosition(1);

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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(55, 188);

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 160;
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