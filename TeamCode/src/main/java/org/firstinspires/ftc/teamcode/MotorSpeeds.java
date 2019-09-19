package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Aryeh on 1/1/17
 */

public class MotorSpeeds extends Object {

    //Four Motor Speeds
    public double frontLeft;
    public double frontRight;
    public double backLeft;
    public double backRight;

    //Refernce to motors
    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    public MotorSpeeds() {
        //Create empty motor speeds
    }

    public void updateMotors() {
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motorFrontLeft.setPower(-frontLeft);
        motorFrontRight.setPower(frontRight);
        motorBackLeft.setPower(-backLeft);
        motorBackRight.setPower(backRight);
    }

    public MotorSpeeds(double frontL, double frontR, double backL, double backR) {
        frontLeft = frontL;
        frontRight = frontR;
        backLeft = backL;
        backRight = backR;
    }

    public MotorSpeeds(DcMotor frontL, DcMotor frontR, DcMotor backL, DcMotor backR) {
        motorFrontLeft = frontL;
        motorFrontRight = frontR;
        motorBackLeft = backL;
        motorBackRight = backR;
    }


    public static MotorSpeeds getSpeed(MotionDirections direction) {
        switch (direction) {
            case N:
                return new MotorSpeeds(1.0,1.0,1.0,1.0);
            case S:
                return new MotorSpeeds(-1.0,-1.0,-1.0,-1.0);
            case E:
                return new MotorSpeeds(1.0,-1.0,-1.0,1.0);
            case W:
                return new MotorSpeeds(-1.0,1.0,1.0,-1.0);
            case NE:
                return new MotorSpeeds(1.0,0,0,1.0);
            case NW:
                return new MotorSpeeds(0,1.0,1.0,0);
            case SE:
                return new MotorSpeeds(0,-1.0,-1.0,0);
            case SW:
                return new MotorSpeeds(-1.0,0,0,-1.0);
            case ROTATER:
                return new MotorSpeeds(1.0,-1.0,1.0,-1.0);
            case ROTATEL:
                return new MotorSpeeds(-1.0,1.0,-1.0,1.0);
            case STOP:
                return new MotorSpeeds(0,0,0,0);
        }

        //Should never run
        return null;
    }

    public void setSpeeds(double fl, double fr, double bl, double br) {
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
    }
    public void setSpeedsFromDirection(MotionDirections direction) {
        MotorSpeeds tempSpeed = MotorSpeeds.getSpeed(direction);
        this.setSpeedsFromMotorSpeeds(tempSpeed);
    }
    public void scaleSpeeds(float scaller) {
        this.frontLeft *= scaller;
        this.frontRight *= scaller;
        this.backLeft *= scaller;
        this.backRight *= scaller;

    }
    public void setSpeedsFromMotorSpeeds(MotorSpeeds speeds) {
        this.frontLeft = speeds.frontLeft;
        this.frontRight = speeds.frontRight;
        this.backLeft = speeds.backLeft;
        this.backRight = speeds.backRight;
    }

    public void rotateLeft() {
        setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.ROTATEL));
        updateMotors();
    }
    public void rotateRight() {
        setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.ROTATER));
        updateMotors();
    }
    public void myStop() {
        setSpeedsFromMotorSpeeds(MotorSpeeds.getSpeed(MotionDirections.STOP));
        updateMotors();
    }
    public static void hardwareSetup(LinearOpMode mode) {

    }




}


enum MotionDirections {
    N,
    S,
    E,
    W,
    NE,
    NW,
    SE,
    SW,
    ROTATER,
    ROTATEL,
    STOP
}
