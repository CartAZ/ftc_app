package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="TeamTankDrive	", group="Test")

public class TeamTankDrive extends OpMode {

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null; //if you see this slap yourself
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    private DcMotor arm = null; //the arm that captures the blocks and balls, controlled by left hand motor
    private DcMotor armActivator = null;

    boolean bDebug = false;

    @Override
    public void init() {

        try {
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);


            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            arm = hardwareMap.get(DcMotor.class, "arm");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotor.Direction.REVERSE);

            armActivator = hardwareMap.get(DcMotor.class, "armActivator");
            armActivator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (IllegalArgumentException iax) {
            bDebug = true;
        }
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;
        float rightTrigger1 = gamepad1.right_trigger;
        float leftTrigger1 = gamepad1.left_trigger;
        boolean leftBumper1 = gamepad1.left_bumper;
        boolean rightBumper1 = gamepad1.right_bumper;

        float rightTrigger2 = gamepad2.right_trigger;  //Second Controller
        float leftTrigger2 = gamepad2.left_trigger;
        boolean leftBumper2 = gamepad2.left_bumper;
        boolean rightBumper2 = gamepad2.right_bumper;

        boolean aButton = gamepad1.a;

        String speed = "Max Speed";

        boolean aIsPressed = false;

        // clip the right/left values so that the values never exceed +/- 1
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left = (float)scaleInput(left);
        right = (float)scaleInput(right);

        if(aButton == true && aIsPressed == false){
            if(speed == "Max Speed"){
                speed = "Quarter Speed";
                aIsPressed = true;
            }
            else if(speed == "Quarter Speed"){
                speed = "Half Speed";
                aIsPressed = true;
            }
            else if(speed == "Half Speed"){
                speed = "Max Speed";
                aIsPressed = true;
            }
        }
        else if(aButton == false && aIsPressed == true){
            aIsPressed = false;
        }

        // write the values to the motors - for now, front and back motors on each side are set the same
        if (!bDebug) {

		/*	if(speed == "Max Speed") {
				rightfrontDrive.setPower(right);
				rightbackDrive.setPower(right);
				leftfrontDrive.setPower(left);
				leftbackDrive.setPower(left);
			}
			else if(speed == "Half Speed"){
				rightfrontDrive.setPower(right/2);
				rightbackDrive.setPower(right/2);
				leftfrontDrive.setPower(left/2);
				leftbackDrive.setPower(left/2);
			}
			else if(speed == "Quarter Speed"){
				rightfrontDrive.setPower(right/4);
				rightbackDrive.setPower(right/4);
				leftfrontDrive.setPower(left/4);
				leftbackDrive.setPower(left/4);
			} */

            rightfrontDrive.setPower(right);
            rightbackDrive.setPower(right);
            leftfrontDrive.setPower(left);
            leftbackDrive.setPower(left);

            if (rightBumper1 == true) {
                armActivator.setPower(1);
            }
            else if (leftBumper1 == true) {
                armActivator.setPower(-1);
            }
            else{
                armActivator.setPower(0);
            }

            if (rightTrigger1 > 0) {
                arm.setPower(rightTrigger1);
            }
            else if (leftTrigger1 > 0) {
                arm.setPower(-leftTrigger1);
            }
            else{
                arm.setPower(0);
            }

			/*
			if(rightBumper2 == true){
				armActivator.setPower(.1);
			}

			else if(leftBumper2 == true){
				armActivator.setPower(-.1);
			}

			if(rightTrigger2 > 0){
				arm.setPower(rightTrigger2);
			}
			else if(leftTrigger2 > 0){
				arm.setPower(-rightTrigger2);
			} */
        }

        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */

        telemetry.addData("Speed: ", speed);
        telemetry.addData("left pwr", String.format("%.2f", left));
        telemetry.addData("right pwr", String.format("%.2f", right));
        telemetry.addData("gamepad1", gamepad1);
        telemetry.addData("gamepad2", gamepad2);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        return dVal*dVal*dVal;		// maps {-1,1} -> {-1,1}
    }

}
