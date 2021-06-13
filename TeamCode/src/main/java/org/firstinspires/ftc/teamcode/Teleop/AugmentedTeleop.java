package org.firstinspires.ftc.teamcode.Teleop;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.Mode;

import static org.firstinspires.ftc.teamcode.drive.PoseStorage.currentPose;

@Disabled
@TeleOp(name = "AugmentedTeleop", group = "TestBot")
public class AugmentedTeleop extends LinearOpMode {


    String shooter = "shooter";
    String intake = "intake";

    String mover = "mover";
    String wobble = "wobble";
    String wobble_motor = "wobble_motor";


    String FrontLeftMotor = "front_left";
    String FrontRightMotor = "front_right";
    String BackLefttMotor = "back_left";
    String BackRightMotor = "back_right";

String extend = "extendor";
    //  String turnArmName = "";

    //  Servo liftServo = hardwareMap.servo.get(liftServoName);


     DcMotorEx shooter1 = null;
    DcMotor ringIntake = null;
Servo extend1 = null;
    Servo pusher = null;
    Servo wobbler = null;
    DcMotor wobbler_motor = null;
    DcMotor frontLeft = null;
    DcMotor frontRight = null;
    DcMotor backLeft = null;
    DcMotor backRight = null;

    Servo locker = null;
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;




    Gamepad g1 = gamepad1;


    @Override
    public void runOpMode() throws InterruptedException {
        String wobble = "wobble";
        String wobble_motor = "wobble_motor";

        String shooter = "shooter";
        String intake = "intake";

        String mover = "mover";
      String extend = "extend";


        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ringIntake = hardwareMap.dcMotor.get(intake);
        pusher = hardwareMap.servo.get(mover);
        wobbler = hardwareMap.servo.get(wobble);
        wobbler_motor = hardwareMap.dcMotor.get(wobble_motor);
        locker = hardwareMap.servo.get("locker");
        extend1 = hardwareMap.servo.get(extend);

        
        boolean currentStateOfT1 = false;
        boolean currentStateOfT2 = false;
        boolean currentStateOfDR = false;
        boolean currentStateOfDL = false;
        boolean currentStateOfDM = false;

        boolean currentStateA2 = false;
        boolean currentB = false;
        boolean currentY = false;
        boolean currentB2 = false;
        boolean currentY2 = false;
        boolean currentX = false;
        boolean currentStateOfB1 = false;
        boolean currentStateOfB2 = false;
        
        boolean y = false;
        boolean x = false;

        Gamepad g1 = gamepad1;
        Gamepad g2 = gamepad2;

        double startTime = 0;

        boolean y2 = false;
        boolean x2 = false;
        
        
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        
        drive.setPoseEstimate(currentPose);
        Pose2d newPose = new Pose2d(0,0,Math.toRadians(180));
        
        waitForStart();
        while (opModeIsActive()) {

            currentStateOfT1 = g2.left_bumper;
            currentStateOfT2 = g2.right_bumper;
            currentStateOfDR = g2.dpad_right;
            currentStateOfDL = g2.dpad_left;
            currentStateOfDM = g1.dpad_down;
            currentStateA2 = g1.a;
            currentStateOfB1=g1.left_bumper;
            currentStateOfB2= g1.right_bumper;
            currentB = g1.b;
            currentY = g1.y;
            currentB2 = g2.b;
            currentY2 = g2.y;
            currentX = g1.x;
            if(currentStateOfDM)
            {
                drive.setPoseEstimate(newPose);
            }
            else
            {
                ;
            }

            drive.update();

            // Read pose
            Pose2d myPose = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Shooter speed", shooter1.getVelocity());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                	drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                	ringIntake.setPower(-g1.right_trigger + g1.left_trigger);
//                	ringIntake.setPower(g1.left_trigger);
                    // ringIntake.setPower(g1.left_trigger);
                    wobbler_motor.setPower(0.7*g2.right_stick_y);

                    if(g2.right_trigger==1) {
                        shooter1.setVelocity(-0.98*g2.left_stick_y *2450 );
                    }
                    else if(g2.left_trigger==1) {
                        shooter1.setVelocity(-0.95*g2.left_stick_y *2450 );
                    }
                    else {
                    shooter1.setVelocity(-g2.left_stick_y*2450);
                    }
    if(g2.x)
    {
        extend1.setPosition(0.4);

    }

                    if (g2.a) {
                        pusher.setPosition(0.3);
                    }
                    else{
                        pusher.setPosition(0.55);
                    }

                    if(g2.left_bumper && currentStateOfT1 != false) {
                        wobbler.setPosition(0.3);
                    }
                    if(g1.right_bumper && currentStateOfB2!=false)
                    {
                        extend1.setPosition(0);
                    }
                    if(g1.left_bumper && currentStateOfB1!=false)
                    {
                        extend1.setPosition(0.9);
                    }

                    if (g2.right_bumper && currentStateOfT2 != false){
                        wobbler.setPosition(1);
                    }
                    if (g2.dpad_right && currentStateOfDR != false){
                        locker.setPosition(1);
                    }
                    if (g2.dpad_left && currentStateOfDL != false){
                        locker.setPosition(0);
                    }
                    if (g1.x && currentX != false){
                        Trajectory toShoot = drive.trajectoryBuilder(myPose)
                                .lineToLinearHeading(new Pose2d(60,-20,Math.toRadians(180)))
                                .build();

                        drive.followTrajectoryAsync(toShoot);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    
                    if (g1.a && currentStateA2 != false){
                    	Trajectory toShoot = drive.trajectoryBuilder(myPose)
                    			.lineToLinearHeading(new Pose2d(0,0,Math.toRadians(180)))
                    			.build();
                    	
                    	drive.followTrajectoryAsync(toShoot);
                    	
                    	currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                     else if (g1.y && currentY != false) {
                     	Trajectory toPowers1 = drive.trajectoryBuilder(myPose)
                    			.lineToLinearHeading(new Pose2d(0,-16,Math.toRadians(180)))
                    			
                    			.build();
                    	Trajectory toPowers2 = drive.trajectoryBuilder(toPowers1.end())
                    			.lineToLinearHeading(new Pose2d(0,-22,Math.toRadians(180)))
                    			.build();
                    	Trajectory toPowers3 = drive.trajectoryBuilder(toPowers2.end())
                    			.lineToLinearHeading(new Pose2d(0,-28,Math.toRadians(180)))
                    			
                    			.build();
                    	
                    	shooter1.setVelocity(2450*0.98);
                    	drive.followTrajectoryAsync(toPowers1);
                    //	ReverseJoystickDrive(0,0,0,0);
                    	sleep(200);
                    	pusher.setPosition(0.3);
                        sleep(400);
                        pusher.setPosition(0.55);
                        sleep(400);

                        shooter1.setVelocity(2450*0.97);
                        drive.followTrajectoryAsync(toPowers2);
                    //    ReverseJoystickDrive(0,0,0,0);
                        sleep(200);
                        pusher.setPosition(0.3);
                        sleep(400);
                        pusher.setPosition(0.55);
                        sleep(400);

                        shooter1.setVelocity(2450*0.96);
                        drive.followTrajectoryAsync(toPowers3);
                  //      ReverseJoystickDrive(0,0,
                   //             0,0);
                        sleep(400);
                        pusher.setPosition(0.3);
                        sleep(400);
                        pusher.setPosition(0.55);
                        sleep(200);
                    	
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                     else if (g2.y && currentY2 != false) {
                         shooter1.setVelocity(2450*0.98);
                        pusher.setPosition(0.3);
                        sleep(300);
                        pusher.setPosition(0.55);
                        sleep(300);
                        shooter1.setVelocity(2450*0.98);
                        pusher.setPosition(0.3);
                        sleep(300);
                        pusher.setPosition(0.55);

                        shooter1.setVelocity(2450*0.98);
                        sleep(300);
                        pusher.setPosition(0.3);
                        sleep(300);
                        pusher.setPosition(0.55);
                       // sleep(00);
                    	 currentMode = Mode.AUTOMATIC_CONTROL;
                     }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                	 if ((g1.b && currentB != false) || (g2.b && currentB != false)) {
                     	drive.cancelFollowing();
			currentMode = Mode.DRIVER_CONTROL;
             
                     }

                    ringIntake.setPower(-g1.right_trigger);
                    // ringIntake.setPower(g1.left_trigger);
                    wobbler_motor.setPower(0.7
                            *g2.right_stick_y);

                    if(g2.right_trigger==1) {
                        shooter1.setVelocity(-0.98*g2.left_stick_y *2450 );
                    }
                    else if(g2.left_trigger==1) {
                        shooter1.setVelocity(-0.96*g2.left_stick_y *2450 );
                    }
                    else {
                        shooter1.setVelocity(-g2.left_stick_y*2450);
                    }

                    if (g2.a) {
                        pusher.setPosition(0.3);
                    }
                    else{
                        pusher.setPosition(0.55);
                    }

                    if(g2.left_bumper && currentStateOfT1 != false) {
                        wobbler.setPosition(0.3);
                    }

                    if (g2.right_bumper && currentStateOfT2 != false){
                        wobbler.setPosition(1);
                    }
                    if (g2.dpad_right && currentStateOfDR != false){
                        locker.setPosition(1);
                    }
                    if (g2.dpad_left && currentStateOfDL != false){
                        locker.setPosition(0);
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            
            
            
            
            

            
//            if (g2.dpad_up){
//                y=true;
//                startTime = System.currentTimeMillis();
//            }
//            if(System.currentTimeMillis() - startTime > 1000&&y) {
//                y = false;
//            }
//            if (g2.dpad_down){
//                x=true;
//                startTime = System.currentTimeMillis();
//            }
//            if(System.currentTimeMillis() - startTime > 1000&&x){
//                x=false;
//            }


            

            
           
            
            
//            if(y){
//                   wobbler_motor.setPower(1);
//
//            }
//            else{
//                wobbler_motor.setPower(0);


//            }
//            if(x){
//                wobbler_motor.setPower(-0.8);
//            }
//            else{
//                wobbler_motor.setPower(0);
//            }
            

        }


    }


    public void JoystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY){
        double forward = leftStickY;
        double right = -leftStickX;
        double clockwise = -rightStickX;

        double frontLeft2 = (forward + clockwise + right);
        double frontRight2 = (forward - clockwise - right);
        double backLeft2 = (forward + clockwise - right);
        double backRight2 = (forward - clockwise + right);

        frontRight.setPower(0.6*frontRight2);
        frontLeft.setPower(0.6*frontLeft2);
        backRight.setPower(0.6*backRight2);
        backLeft.setPower(0.6*backLeft2);
    }

    public void ReverseJoystickDrive(double leftStickX, double leftStickY, double rightStickX, double rightStickY){
        double forward = leftStickY;
        double right = -leftStickX;
        double clockwise = -rightStickX;

        double frontLeft2 = (-forward - clockwise - right);
        double frontRight2 = (-forward + clockwise + right);
        double backLeft2 = (-forward - clockwise + right);
        double backRight2 = (-forward + clockwise - right);

        frontRight.setPower(0.7*frontRight2);
        frontLeft.setPower(0.7*frontLeft2);
        backRight.setPower(0.7*backRight2);
        backLeft.setPower(0.7*backLeft2);
    }
    
    public void sleep(double millis) {
    	double startTime = System.currentTimeMillis();
    	while(opModeIsActive() && System.currentTimeMillis() - startTime < millis);
    	return;
    }

}



//      drop();
//this is for checking red
//  if (colorSensor.red() > 200 && colorSensor.blue() > 200 && colorSensor.green() > 200) {
//     setDirectionForward(false, 1000);
//   turn90(false,2000);
//turn90(true,500);
//setDirectionForward(true, 400);

//                if (colorSensor.red() > 200 && colorSensor.blue() > 200 && colorSensor.green() > 200) {
//                    setDirectionForward(false, 1000);
//                    turn90(false,500);
//                    turn90(true,500);
//                    setDirectionForward(true,400);
//
//                }
//                else if (colorSensor.red() > 200 && colorSensor.blue() < 100 && colorSensor.green() > 200) {
//                    setDirectionForward(true,400);
//
//                }
//            }-
//            else if (colorSensor.red() > 200 && colorSensor.blue() < 100 && colorSensor.green() > 200) {
//                setDirectionForward(true, 400);
//
//     }
//   }


