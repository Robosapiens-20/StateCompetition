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

package org.firstinspires.ftc.teamcode.Debugs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name = "OpenCVDebug1")
public class control_debug extends LinearOpMode {
    OpenCvCamera camera;
    SkystoneDeterminationPipeline pipeline;
    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotorEx shooter = null;
    Servo pusher = null;
    DcMotor intake = null;
    WebcamName webcamName = null;
    Gamepad g1 = new Gamepad();

    @Override
    public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
//        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pusher = hardwareMap.servo.get("mover");
        intake = hardwareMap.dcMotor.get("intake");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        camera.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //   camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }
        });



            while(!opModeIsActive())
            {
                if (g1.dpad_up)
                pipeline.FOUR_RING_THRESHOLD = pipeline.FOUR_RING_THRESHOLD + 1;
                if(g1.dpad_down)
                    pipeline.FOUR_RING_THRESHOLD = pipeline.FOUR_RING_THRESHOLD - 1;
                if(g1.dpad_left)
                    pipeline.ONE_RING_THRESHOLD = pipeline.ONE_RING_THRESHOLD+1;
                if(g1.dpad_right)
                    pipeline.ONE_RING_THRESHOLD = pipeline.ONE_RING_THRESHOLD-1;

                if(g1.a){ telemetry.addData("Analysis", pipeline.getAnalysis());
                    telemetry.addData("Position", pipeline.position);}
            }


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();


            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                telemetry.addData("Position 1", pipeline.position);
            } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                telemetry.addData("Position 4", pipeline.position);
            } else {
                telemetry.addData("Position 0", pipeline.position);
            }

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

        public int FOUR_RING_THRESHOLD = 160;
        public int ONE_RING_THRESHOLD = 130;

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