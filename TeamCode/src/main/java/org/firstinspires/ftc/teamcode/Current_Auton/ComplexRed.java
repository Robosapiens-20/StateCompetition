package org.firstinspires.ftc.teamcode.Current_Auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Vector;

@Autonomous(name = "States Red Complex")
public class ComplexRed extends LinearOpMode {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 3, 13);
    SampleMecanumDrive drive;

    OpenCvCamera phoneCam;
    WebcamName webcamName;
    SkystoneDeterminationPipeline pipeline;

    DcMotor wobbler_motor = null;
    Servo wobble_servo = null;
    Servo wobble_servo2;
    DcMotorEx shooter = null;
    Servo pusher = null;
    DcMotor intake = null;

    int shooterPower = Constants.shooterPower;

    double firstWobble = 2.2;
    double secondWobble = 0.9;
    double thirdWobble = 1.7;

    double intakePower = -1;

    Trajectory toDepot = null;
    Trajectory toDepot2 = null;
    Trajectory backwards = null;
    Trajectory toShoot = null;
    Trajectory position = null;
    Trajectory intakeRings = null;
    Trajectory toShoot2 = null;
    Trajectory park = null;
    Trajectory toWobble1 = null;
    Trajectory toWobble2 = null;


    Pose2d shootingPosition = new Pose2d(64, 12, Math.toRadians(180));

    public void oneRing() {
        toDepot = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(40, -5), Math.toRadians(0))
                .splineTo(new Vector2d(80, 15), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1);
                    wobble_servo2.setPosition(1);
                    wobbler_motor.setPower(0.5);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                    pusher.setPosition(Constants.backShooterServo);
                })
                .build();

        toShoot = drive.trajectoryBuilder(toDepot.end())
                .lineToLinearHeading(new Pose2d(shootingPosition.getX(), shootingPosition.getY() + 5, shootingPosition.getHeading()))
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);
                    wobbler_motor.setPower(-0.8);
                })
                .addTemporalMarker(secondWobble + 0.2, () -> {
                    wobbler_motor.setPower(0);
                })
                .build();

        position = drive.trajectoryBuilder(toShoot.end())
                .lineToLinearHeading(new Pose2d(62, 18, Math.toRadians(180)))
                .build();

        intakeRings = drive.trajectoryBuilder(position.end())
                .forward(40)
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(intakePower);
                    wobble_servo.setPosition(0);
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(secondWobble + 0.2,()->{
                    wobbler_motor.setPower(0);
                })
                .build();

        double wobbleX = 7;
        toWobble1 = drive.trajectoryBuilder(intakeRings.end())
                .lineToLinearHeading(new Pose2d(wobbleX, -10, Math.toRadians(-90)))
                .addTemporalMarker(0.8, () -> {
                    intake.setPower(-1);
                })
                .build();
        toWobble2 = drive.trajectoryBuilder(toWobble1.end())
                .lineToLinearHeading(new Pose2d(wobbleX, 10, Math.toRadians(-90)))
                .addTemporalMarker(0.99, 0, () -> {
                    wobble_servo.setPosition(1);
                    wobble_servo2.setPosition(1);
                })
                .build();

        toShoot2 = drive.trajectoryBuilder(toWobble2.end())
                .lineToLinearHeading(new Pose2d(shootingPosition.getX() - 10, shootingPosition.getY() + 5, shootingPosition.getHeading()))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(-0.8);
                })
                .addTemporalMarker(0.3, () -> {
                    wobbler_motor.setPower(0);
                })
                .build();

        toDepot2 = drive.trajectoryBuilder(toShoot2.end())
                .lineToLinearHeading(new Pose2d(85, 25, Math.toRadians(180)))
                .addTemporalMarker(0.1,()->{
                    intake.setPower(0);
                })
                .addDisplacementMarker(0.95, 0, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                })
                .build();

        park = drive.trajectoryBuilder(toDepot2.end())
                .forward(10)
                .build();
    }

    public void zeroRing() {
        toDepot = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(57, -18), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1);
                    wobble_servo2.setPosition(1);
                    wobbler_motor.setPower(0.5);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                    pusher.setPosition(Constants.backShooterServo);
                })

                .build();
        backwards = drive.trajectoryBuilder(toDepot.end())
                .forward(15)
                .build();

        toShoot = drive.trajectoryBuilder(backwards.end())
                .lineToLinearHeading(shootingPosition)
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);
                    wobbler_motor.setPower(-0.5);
                })
                .addTemporalMarker(thirdWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .build();

        int wobbleY = 15;
        toWobble1 = drive.trajectoryBuilder(toShoot.end())
                .lineToLinearHeading(new Pose2d(60, wobbleY, Math.toRadians(0)))
                .build();

        toWobble2 = drive.trajectoryBuilder(toWobble1.end())
                .lineToLinearHeading(new Pose2d(29, wobbleY, Math.toRadians(0)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(secondWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .addTemporalMarker(0.99,0,()->{
                    wobble_servo.setPosition(1);
                    wobble_servo2.setPosition(1);
                })
                .build();

        toDepot2 = drive.trajectoryBuilder(toWobble2.end())
                .lineToLinearHeading(new Pose2d(85, 3, Math.toRadians(90)))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(-0.8);
                })
                .addTemporalMarker(0.3, () -> {
                    wobbler_motor.setPower(0);
                })
                .addTemporalMarker(0.95,0,()->{
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);

                })
                .build();

        park = drive.trajectoryBuilder(toDepot2.end())
                .lineTo(new Vector2d(80, 25))
                .build();

    }

    public void fourRings() {
        toDepot = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(105, -18), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    wobble_servo.setPosition(1);
                    wobble_servo2.setPosition(1);
                    wobbler_motor.setPower(0.5);
                })
                .addTemporalMarker(firstWobble, () -> {
                    wobbler_motor.setPower(0);
                })
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                    pusher.setPosition(Constants.backShooterServo);
                })
                .build();


        toShoot = drive.trajectoryBuilder(toDepot.end())
                .lineToLinearHeading(shootingPosition)
                .addTemporalMarker(0.1, () -> {
                    shooter.setVelocity(shooterPower);
                    wobbler_motor.setPower(-0.5);
                })

                .addTemporalMarker(thirdWobble, () -> {
                    wobbler_motor.setPower(0);
                })

                .build();

        position = drive.trajectoryBuilder(toShoot.end())
                .lineToLinearHeading(new Pose2d(62, 14, Math.toRadians(180)))
                .build();

        intakeRings = drive.trajectoryBuilder(position.end())
                .forward(50)
                .addTemporalMarker(0.1, () -> {
                    intake.setPower(intakePower);
                    wobble_servo.setPosition(0);
                    wobbler_motor.setPower(0.8);
                })
                .addTemporalMarker(1.1,()->{
                    wobbler_motor.setPower(0);
                })
                .build();

        double wobbleX = 7;
        toWobble1 = drive.trajectoryBuilder(intakeRings.end())
                .lineToLinearHeading(new Pose2d(wobbleX, -15, Math.toRadians(-90)))
                .addTemporalMarker(0.8, () -> {
                    intake.setPower(-1);
                })
                .build();

        toWobble2 = drive.trajectoryBuilder(toWobble1.end())
                .lineToLinearHeading(new Pose2d(wobbleX, 5, Math.toRadians(-90)))
                .addTemporalMarker(0.99, 0, () -> {
                    wobble_servo.setPosition(1);
                    wobble_servo2.setPosition(1);
                })
                .build();

        toShoot2 = drive.trajectoryBuilder(toWobble2.end())
                .lineToLinearHeading(new Pose2d(shootingPosition.getX() - 10, shootingPosition.getY(), shootingPosition.getHeading()))
                .addTemporalMarker(0.1, () -> {
                    wobbler_motor.setPower(-0.8);
                })
                .addTemporalMarker(0.3, () -> {
                    wobbler_motor.setPower(0);
                })
                .addTemporalMarker(0.99, 0, () -> {
                    intake.setPower(0);
                })
                .build();

        toDepot2 = drive.trajectoryBuilder(toShoot2.end())
                .lineToLinearHeading(new Pose2d(110, -17, Math.toRadians(180)))
                .addDisplacementMarker(0.95, 0.5, () -> {
                    wobble_servo.setPosition(0);
                    wobble_servo2.setPosition(0);
                })
                .build();

        park = drive.trajectoryBuilder(toDepot2.end())
                .forward(30)
                .build();

    }

    public void shootRings(int num) {
        for (int i = 1; i <= num; i++) {
            shooter.setVelocity(shooterPower);
            sleep(300);
            pusher.setPosition(0.45);
            sleep(300);
            pusher.setPosition(Constants.backShooterServo);
        }
    }

    public void shootRings(int num, int power) {
        for (int i = 1; i <= num; i++) {
            shooter.setVelocity(power);
            sleep(300);
            pusher.setPosition(0.45);
            sleep(300);
            pusher.setPosition(Constants.backShooterServo);
        }
    }

    @Override
    public void runOpMode() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive = new SampleMecanumDrive(hardwareMap);

        wobbler_motor = hardwareMap.dcMotor.get("wobble_motor");
        wobble_servo = hardwareMap.servo.get("wobble");
        wobble_servo2 = hardwareMap.servo.get("wobble2");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        MotorConfigurationType motorConfigurationType = shooter.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooter.setMotorType(motorConfigurationType);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));

        pusher = hardwareMap.servo.get("mover");
        intake = hardwareMap.dcMotor.get("intake");

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        wobble_servo.setPosition(1);
        wobble_servo2.setPosition(1);
        wobbler_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            if (initial == one)
                oneRing();
            else if (initial == four)
                fourRings();
            else
                zeroRing();

            drive.followTrajectory(toDepot);
            sleep(200);

            if (initial == none)
                drive.followTrajectory(backwards);

            shooter.setVelocity(shooterPower);
            drive.followTrajectory(toShoot);
            shootRings(3);
            shooter.setVelocity(-0.5 * shooterPower);
            if (initial != none) {
                drive.followTrajectory(position);
                drive.followTrajectory(intakeRings);
                drive.followTrajectory(toWobble1);
                drive.followTrajectory(toWobble2);
                shooter.setVelocity(shooterPower);
                drive.followTrajectory(toShoot2);
                shootRings(initial == four ? 4 : 2, 1440);
                shooter.setVelocity(0);
            } else {
                drive.followTrajectory(toWobble1);
                drive.followTrajectory(toWobble2);
            }
            drive.followTrajectory(toDepot2);
            drive.followTrajectory(park);
            wobble_servo.setPosition(0);
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

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(55, 188);

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 153;
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
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = RingPosition.FOUR;
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }
}