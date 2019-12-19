package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;



@TeleOp(name="Manual", group="TeleOp")
public class MecanumTeleop extends LinearOpMode {

    //protected Servo sideStick;
    protected DcMotor motorFrontLeft;
    protected DcMotor motorFrontRight;
    protected DcMotor motorBackLeft;
    protected DcMotor motorBackRight;
    protected DcMotor peretz;
    private Servo grabber1=null;
    private Servo grabber2=null;
    private Servo grabberTilt = null;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 5.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    MotorSpeeds speed;

    @Override
    public void runOpMode() throws InterruptedException {
        //Init
        //Servos
        hardwareSetup();

        speed = new MotorSpeeds(motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight);

        waitForStart();
        //start

        //Extend color senser arm slightly to get out of way
        //MecanumAutonomus.moveDcMotorEncoded(sideStickMotor,0.2,-100);

        //extend relic extender slightly
//        relicExtender.setPower(-0.5);
//        sleep(500);
//        relicExtender.setPower(0.0);


        while (opModeIsActive()) {
            //Loop

            if (Math.abs(gamepad1.right_stick_x) >= 0.01 || Math.abs(gamepad1.right_stick_y) >= 0.01) {
                rightJoystick();
            } else if (gamepad1.left_trigger > 0.01) {
                leftTrigger(gamepad1.left_trigger);
                //Rotate left

            } else if (gamepad1.right_trigger > 0.01) {
                //rotate right
                rightTrigger(gamepad1.right_trigger);

            } else {
                speed.myStop();
            }
            peretz.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
            if(gamepad2.dpad_right) {
                grabber1.setPosition(.7);
                grabber2.setPosition(.7);
            }
            if(gamepad2.dpad_left){
                grabber1.setPosition(.4);
                grabber2.setPosition(.4);
            }
            if(gamepad2.dpad_up){
                grabberTilt.setPosition(.7);
            }
            if(gamepad2.dpad_down){
                grabberTilt.setPosition(.3);
            }





        }
    }

            private void hardwareSetup(){
                //Servos


                //Motors
                motorFrontLeft = hardwareMap.dcMotor.get("fl");
                motorFrontRight = hardwareMap.dcMotor.get("fr");
                motorBackLeft = hardwareMap.dcMotor.get("bl");
                motorBackRight = hardwareMap.dcMotor.get("br");
                peretz = hardwareMap.dcMotor.get("peretz");

                grabber1= hardwareMap.servo.get("grabber1");
                grabber2= hardwareMap.servo.get("grabber2");
                grabberTilt = hardwareMap.servo.get("grabber_tilt");
                grabber1.setDirection(Servo.Direction.FORWARD);
                grabber2.setDirection(Servo.Direction.REVERSE);
                grabberTilt.setDirection(Servo.Direction.FORWARD);
                peretz.setDirection(DcMotorSimple.Direction.FORWARD);


                grabberTilt.setPosition(.8);
                grabber1.setPosition(0);
                grabber2.setPosition(0);

            }

            private void rightTrigger ( double triggerValue){
                speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.ROTATER));

                speed.backRight *= triggerValue;
                speed.backLeft *= triggerValue;
                speed.frontLeft *= triggerValue;
                speed.frontRight *= triggerValue;
                speed.updateMotors();

            }
            private void leftTrigger ( double triggerValue){
                speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.ROTATEL));

                speed.backRight *= triggerValue;
                speed.backLeft *= triggerValue;
                speed.frontLeft *= triggerValue;
                speed.frontRight *= triggerValue;
                speed.updateMotors();


            }


            private void rightJoystick () {
                double fixedYValue = -gamepad1.right_stick_y;
                double xValue = gamepad1.right_stick_x;


                double rightRotation = gamepad1.right_trigger;
                double leftRotation = gamepad1.left_trigger;


                double robotSpeed = Math.sqrt(Math.pow(fixedYValue, 2) + Math.pow(xValue, 2));
                double changeDirectionSpeed = 0;

                if (rightRotation > leftRotation) {
                    changeDirectionSpeed = rightRotation;
                } else {
                    changeDirectionSpeed = -leftRotation;
                }


                double frontLeftPower = robotSpeed * Math.sin(Math.atan2(xValue, fixedYValue) + Math.PI / 4) + changeDirectionSpeed;
                double frontRightPower = robotSpeed * Math.cos(Math.atan2(xValue, fixedYValue) + Math.PI / 4) - changeDirectionSpeed;
                double backLeftPower = robotSpeed * Math.cos(Math.atan2(xValue, fixedYValue) + Math.PI / 4) + changeDirectionSpeed;
                double backRightPower = robotSpeed * Math.sin(Math.atan2(xValue, fixedYValue) + Math.PI / 4) - changeDirectionSpeed;


                //public MotorSpeeds(double frontL, double frontR, double backL, double backR)

                speed.setSpeeds(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
                speed.frontLeft = speed.frontLeft;
                speed.frontRight = speed.frontRight;
                speed.backLeft = speed.backLeft;
                speed.backRight = speed.backRight;
                speed.updateMotors();


            }
    public void encoderDrive(double speed,
                             double peretzz,
                             double timeoutS) {


        int newPeretzTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newPeretzTarget = peretz.getCurrentPosition() + (int) (peretzz * COUNTS_PER_INCH);
            peretz.setTargetPosition(newPeretzTarget);

            // Turn On RUN_TO_POSITION
            peretz.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            peretz.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (peretz.isBusy())) {
                int jim = peretz.getCurrentPosition();
                telemetry.addData("Position: ",newPeretzTarget);
                telemetry.update();
            }
            }
                // Stop all motion;
                peretz.setPower(0);

                // Turn off RUN_TO_POSITION
                peretz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }





