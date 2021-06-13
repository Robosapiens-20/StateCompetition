package org.firstinspires.ftc.teamcode.Current_Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "States Red Powershots")

public class RedPowerShots extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 3, 13);
    OpenCvCamera phoneCam;
    WebcamName webcamName;
    SkystoneDeterminationPipeline pipeline;

    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    DcMotorEx shooter = null;
    Servo pusher = null;
    Servo wobble_servo2;
    DcMotor intake = null;

    double firstWobble = 0.9;
    int shooterPower = Constants.powerShotPower;
    int sleepTime = 3000;

    SampleMecanumDrive drive;

    Trajectory outOfTheWay;
    Trajectory rightShot;
    Trajectory middleShot;
    Trajectory leftShot;

    Trajectory toDepot1;
    Trajectory toDepot2;
    Trajectory toDepot3;
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;

    public void hitPowershots() {
        outOfTheWay = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 12, Math.toRadians(0)))
                .addTemporalMarker(0.1, () -> {
                    pusher.setPosition(Constants.backShooterServo);
                })
                .build();

        leftShot = drive.trajectoryBuilder(outOfTheWay.end())
                .lineToLinearHeading(new Pose2d(62, 20, Math.toRadians(180)))
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);
                })
                .build();

        middleShot = drive.trajectoryBuilder(leftShot.end())
                .lineToLinearHeading(new Pose2d(62, 11, Math.toRadians(180)))
                .build();

        rightShot = drive.trajectoryBuilder(middleShot.end())
                .lineToLinearHeading(new Pose2d(62, 2.5, Math.toRadians(180)))
                .build();
    }

    public void oneRing() {
        toDepot1 = drive.trajectoryBuilder(rightShot.end())
                .lineToLinearHeading(new Pose2d(105, 2, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                })
                .build();

        park1 = drive.trajectoryBuilder(toDepot1.end())
                .forward(20)
                .build();

        park2 = drive.trajectoryBuilder(park1.end())
                .strafeLeft(30)
                .build();
    }

    public void zeroRing() {
        toDepot1 = drive.trajectoryBuilder(rightShot.end())
                .lineToLinearHeading(new Pose2d(115, 12, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                    pusher.setPosition(Constants.backShooterServo);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .build();

        toDepot2 = drive.trajectoryBuilder(toDepot1.end())
                .lineToLinearHeading(new Pose2d(115, -33, Math.toRadians(0)))
                .build();

        toDepot3 = drive.trajectoryBuilder(toDepot2.end())
                .lineToLinearHeading(new Pose2d(90, -33, Math.toRadians(0)))
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                })
                .build();

        park1 = drive.trajectoryBuilder(toDepot3.end())
                .forward(20)
                .build();

        park2 = drive.trajectoryBuilder(park1.end())
                .strafeLeft(40)
                .build();

        park3 = drive.trajectoryBuilder(park2.end())
                .back(30)
                .build();
    }

    public void fourRings() {
        toDepot1 = drive.trajectoryBuilder(rightShot.end())
                .lineToLinearHeading(new Pose2d(122, 12, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .build();

        toDepot2 = drive.trajectoryBuilder(toDepot1.end())
                .lineToLinearHeading(new Pose2d(122, -18, Math.toRadians(90)))
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                })
                .build();

        park1 = drive.trajectoryBuilder(toDepot2.end())
                .forward(40)
                .build();

        park2 = drive.trajectoryBuilder(park1.end())
                .strafeLeft(40)
                .build();
    }

    public void shootRings(int num) {
        for (int i = 1; i <= num; i++) {
            shooter.setVelocity(shooterPower);
            sleep(300);
            pusher.setPosition(Constants.frontShooterServo);
            sleep(300);
            pusher.setPosition(Constants.backShooterServo);
        }
    }

    public void shootRings(int num, int power) {
        shooter.setVelocity(power);
        sleep(200);
        for (int i = 1; i <= num; i++) {
            sleep(300);
            pusher.setPosition(Constants.frontShooterServo);
            sleep(300);
            pusher.setPosition(Constants.backShooterServo);
        }
    }

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
        wobble_servo2 = hardwareMap.servo.get("wobble2");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pusher = hardwareMap.servo.get("mover");
        intake = hardwareMap.dcMotor.get("intake");

        wobble_servo.setPosition(1);
        wobble_servo2.setPosition(1);
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        sleep(1500);
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            for (int i = 0; i < 2; i++) {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Position", pipeline.position);
                telemetry.update();
                sleep(50);
            }

            SkystoneDeterminationPipeline.RingPosition initial = pipeline.position;
            SkystoneDeterminationPipeline.RingPosition one = SkystoneDeterminationPipeline.RingPosition.ONE;
            SkystoneDeterminationPipeline.RingPosition four = SkystoneDeterminationPipeline.RingPosition.FOUR;
            SkystoneDeterminationPipeline.RingPosition none = SkystoneDeterminationPipeline.RingPosition.NONE;

            hitPowershots();

            sleep(sleepTime);

            drive.followTrajectory(outOfTheWay);
            shooter.setVelocity(shooterPower);
            drive.followTrajectory(leftShot);
            shootRings(1);
            drive.followTrajectory(middleShot);
            shootRings(1, shooterPower - 20);
            drive.followTrajectory(rightShot);
            shootRings(1, shooterPower - 20);
            shooter.setPower(0);

            if (initial == one)
                oneRing();
            else if (initial == four)
                fourRings();
            else
                zeroRing();

            drive.followTrajectory(toDepot1);
            if (initial != one)
                drive.followTrajectory(toDepot2);
            if (initial == none)
                drive.followTrajectory(toDepot3);

            drive.followTrajectory(park1);
            drive.followTrajectory(park2);
            if (initial == none)
                drive.followTrajectory(park3);
            break;
        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(270, 192);

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

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = RingPosition.FOUR;

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