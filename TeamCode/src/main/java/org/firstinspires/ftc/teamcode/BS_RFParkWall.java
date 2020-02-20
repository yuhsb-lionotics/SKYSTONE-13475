package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 *
 * Created (- VuForia) by Zack 9/23/19
 *
 *
 * This 2019-2020 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the SKYSTONE FTC field.
 * The code is structured as a LinearOpMode
 */

@Autonomous(name="BS/RF Park Against Wall", group ="Autonomous")
public class BS_RFParkWall extends LinearOpMode{

    //The following are addition made by your's truly:
    private DcMotor FR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor BR = null;
    private DcMotor peretz = null;
    private Servo grabber1=null;
    private Servo grabber2=null;
    private Servo grabberTilt=null;


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1220;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {
        setUp();
        waitForStart();
/** Let's quickly recap how to use encoder drive. Refer to class MotionSpeeds in MotorSpeeds.java
 *             !These are just motor values, not what encoder drive should be!
 *             case N: encodeDrive(1.0,1.0,1.0,1.0);
 *             case S: encodeDrive(-1.0,-1.0,-1.0,-1.0);
 *             case E: encodeDrive(1.0,-1.0,-1.0,1.0);
 *             case W: encodeDrive(-1.0,1.0,1.0,-1.0);
 *             case NE: encodeDrive(1.0,0,0,1.0);
 *             case NW: encodeDrive(0,1.0,1.0,0);
 *             case SE: encodeDrive(0,-1.0,-1.0,0);
 *             case SW: encodeDrive(-1.0,0,0,-1.0);
 *             case ROTATER: encodeDrive(1.0,-1.0,1.0,-1.0);
 *             case ROTATEL: encodeDrive(-1.0,1.0,-1.0,1.0);
 *             case STOP:encodeDrive(0,0,0,0);
 *
 */

        //CODE GOES HERE- THANK YOU FOR THE INDICATION:
        sleep(25000);
        encoderDrive(1, 18, 18, 18, 18, 0, 5);
    }
    private void setUp() {
        FR = hardwareMap.get(DcMotor.class, "fr");
        FL = hardwareMap.get(DcMotor.class, "fl");
        BR = hardwareMap.get(DcMotor.class, "br");
        BL = hardwareMap.get(DcMotor.class, "bl");
        peretz = hardwareMap.get(DcMotor.class, "peretz");
        grabber1= hardwareMap.servo.get("grabber1");
        grabber2= hardwareMap.servo.get("grabber2");
        grabberTilt= hardwareMap.servo.get("grabber_tilt");



        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        peretz.setDirection(DcMotor.Direction.FORWARD);
        grabber1.setDirection(Servo.Direction.FORWARD);
        grabber2.setDirection(Servo.Direction.REVERSE);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        peretz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        peretz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        grabber1.setPosition(.1);
        grabber2.setPosition(.1);
        grabberTilt.setPosition(1);

        telemetry.addData("Game Time", "Yee");
        telemetry.update();

    }

    public void encoderDrive(double speed,
                             double FLin, double FRin, double BLin, double BRin, double peretzz,
                             double timeoutS) {
        int newFLTarget;
        int newFRTarget;
        int newBLTarget;
        int newBRTarget;
        int newPeretzTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLTarget = FL.getCurrentPosition() + (int) (FLin * COUNTS_PER_INCH);
            newFRTarget = FR.getCurrentPosition() + (int) (FRin * COUNTS_PER_INCH);
            newBLTarget = BL.getCurrentPosition() + (int) (BLin * COUNTS_PER_INCH);
            newBRTarget = BR.getCurrentPosition() + (int) (BRin * COUNTS_PER_INCH);
            newPeretzTarget = peretz.getCurrentPosition() + (int) (peretzz*COUNTS_PER_INCH);

            FR.setTargetPosition(newFRTarget);
            FL.setTargetPosition(newFLTarget);
            BL.setTargetPosition(newBLTarget);
            BR.setTargetPosition(newBRTarget);
            peretz.setTargetPosition(newPeretzTarget);

            // Turn On RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            peretz.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FR.setPower(Math.abs(speed));
            FL.setPower(Math.abs(speed));
            BL.setPower(Math.abs(speed));
            BR.setPower(Math.abs(speed));
            peretz.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    ((FR.isBusy() && FL.isBusy() && BL.isBusy() && BR.isBusy())||peretz.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d", newFLTarget, newFRTarget, newBLTarget, newFLTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d",
                        FR.getCurrentPosition(),
                        FL.getCurrentPosition(),
                        BL.getCurrentPosition(),
                        BR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FR.setPower(0);
            FL.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            peretz.setPower(0);

            // Turn off RUN_TO_POSITION
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            peretz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


}
