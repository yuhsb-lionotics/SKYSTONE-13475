package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private Servo grabber1=null;
    private Servo grabber2=null;
    private Servo grabberTilt = null;

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
            if(gamepad1.b) {
                grabber1.setPosition(.7);
                grabber2.setPosition(.7);
            }
            if(gamepad1.x){
                grabber1.setPosition(.4);
                grabber2.setPosition(.4);
            }
            if(gamepad1.y){
                grabberTilt.setPosition(.6);
            }
            if(gamepad1.a){
                grabberTilt.setPosition(.1);
            }
            if(gamepad1.dpad_up){
                grabberTilt.setPosition(7);
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

                grabber1= hardwareMap.servo.get("grabber1");
                grabber2= hardwareMap.servo.get("grabber2");
                grabberTilt = hardwareMap.servo.get("grabber_tilt");
                grabber1.setDirection(Servo.Direction.FORWARD);
                grabber2.setDirection(Servo.Direction.REVERSE);
                grabberTilt.setDirection(Servo.Direction.FORWARD);

                grabberTilt.setPosition(.9);
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




        /*
        //Speed is a double from 0 - 1.0 and is a scale factor to be applied to motor powers
        double speedCoef = Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2));

        if(gamepad1.left_stick_x >= 0 && fixedYValue >= 0) {
            //first quad

            //If diff of values is less than .3 go diagonal
            if(Math.abs(gamepad1.left_stick_x - fixedYValue) > 0.3) {
                //Go horizontal or vertical

                //X > Y GO EAST
                if(gamepad1.left_stick_x > fixedYValue) speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.E));

                    //Y > X GO NORTH
                else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.N));
            }

            else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.NE));
        }

        else if(gamepad1.left_stick_x < 0 && fixedYValue >= 0) {
            //second quad
            if(Math.abs(-gamepad1.left_stick_x - fixedYValue) > 0.3) {
                if(-gamepad1.left_stick_x > fixedYValue) {
                    //GO W
                    speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.W));
                }

                else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.N));
            }
            else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.NW));

        }
        else if(gamepad1.left_stick_x < 0 && fixedYValue < 0) {
            //third quad

            if(Math.abs(gamepad1.left_stick_x - fixedYValue) > 0.3) {
                if(gamepad1.left_stick_x < fixedYValue) {
                    //GO W
                    speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.W));
                }

                else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.S));
            }
            else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.SW));

        }
        else {
            //fourth quad

            if(Math.abs(gamepad1.left_stick_x + fixedYValue) > 0.3) {
                if(gamepad1.left_stick_x > -fixedYValue) {
                    //GO W
                    speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.E));
                }

                else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.S));
            }
            else speed.setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.SE));

        }

        //apply speed coefficient
        speed.backRight *= speedCoef;
        speed.backLeft *= speedCoef;
        speed.frontRight *= speedCoef;
        speed.frontLeft *= speedCoef;

        speed.updateMotors();
*/

            }

        }



