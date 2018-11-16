package org.firstinspires.ftc.teamcode._TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="KartSimpleTankDrive", group="Test")

public class KartSimpleTankDrive extends OpMode {

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    private DcMotor armLeft = null;
    private DcMotor armRight = null;

    boolean bDebug = false;

    private BNO055IMU imu;

    private Servo markerArm;

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
            rightbackDrive.setDirection(DcMotor.Direction.REVERSE);

            armLeft = hardwareMap.get(DcMotor.class, "armActivatorLeft");
            armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armRight = hardwareMap.get(DcMotor.class, "armActivatorRight");
            armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRight.setDirection(DcMotor.Direction.REVERSE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");
            markerArm.setPosition(0);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        catch (IllegalArgumentException iax) {
            bDebug = true;
        }
    }

    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
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

        float rightTrigger = gamepad1.right_trigger;

        float leftTrigger = gamepad1.left_trigger;
        boolean leftBumper = gamepad1.left_bumper;

        // clip the right/left values so that the values never exceed +/- 1
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left = (float)scaleInput(left);
        right = (float)scaleInput(right);

        // write the values to the motors - for now, front and back motors on each side are set the same
        if (!bDebug) {

            rightfrontDrive.setPower(right);
            rightbackDrive.setPower(right);
            leftfrontDrive.setPower(left);
            leftbackDrive.setPower(left);

            if(leftTrigger == 1) {
                armLeft.setPower(1);
                armRight.setPower(1);
            }
            if(leftBumper == true){
                armLeft.setPower(1);
                armRight.setPower(1);
            }

            if(rightTrigger == 1){
                markerArm.setPosition(1);
            }
        }



        /*
         * Send telemetry data back to driver station. Note that if we are using
         * a legacy NXT-compatible motor controller, then the getPower() method
         * will return a null value. The legacy NXT-compatible motor controllers
         * are currently write only.
         */

        telemetry.addData("left pwr", String.format("%.2f", left));
        telemetry.addData("right pwr", String.format("%.2f", right));
        telemetry.addData("gamepad1", gamepad1);
        telemetry.addData("gamepad2", gamepad2);
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addLine()
                .addData("status", imu.getSystemStatus().toShortString())
                .addData("calib", imu.getCalibrationStatus().toString())
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
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

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
