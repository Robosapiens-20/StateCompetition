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
import org.firstinspires.ftc.teamcode.Current_Auton.Constants;
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

@TeleOp(name = "OpenCVDebugBlue")
public class OpenCVDebug extends LinearOpMode {
    OpenCvCamera camera;
    SkystoneDeterminationPipeline pipeline;
    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotorEx shooter = null;
    Servo pusher = null;
    DcMotor intake = null;
    WebcamName webcamName = null;

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


        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);

        boolean isDownDown = false;
        boolean isDownUp = false;
        boolean isDownDown2 = false;
        boolean isDownUp2 = false;

        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            isDownDown = g1.dpad_down;
            isDownUp = g1.dpad_up;
            isDownDown2 = g2.dpad_down;
            isDownUp2 = g2.dpad_up;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            if (gamepad1.dpad_down && isDownDown) {
                Constants.oneRingThresholdBlue--;
                pipeline.ONE_RING_THRESHOLD = Constants.oneRingThresholdBlue;
            }
            if (gamepad1.dpad_up && isDownUp) {
                Constants.oneRingThresholdBlue++;
                pipeline.ONE_RING_THRESHOLD = Constants.oneRingThresholdBlue;
            }
            if (gamepad2.dpad_down && isDownDown2) {
                Constants.fourRingThresholdBlue--;
                pipeline.FOUR_RING_THRESHOLD = Constants.fourRingThresholdBlue;
            }
            if (gamepad2.dpad_up && isDownUp2) {
                Constants.fourRingThresholdBlue++;
                pipeline.FOUR_RING_THRESHOLD = Constants.fourRingThresholdBlue;
            }


            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                telemetry.addData("Position 1", pipeline.position);
            } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                telemetry.addData("Position 4", pipeline.position);
            } else {
                telemetry.addData("Position 0", pipeline.position);
            }
            sleep(50);

        }
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

        int FOUR_RING_THRESHOLD = Constants.fourRingThresholdBlue;
        int ONE_RING_THRESHOLD = Constants.oneRingThresholdBlue;

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
