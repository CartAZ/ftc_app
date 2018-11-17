package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="TeamLeftAuto", group="Autonomous")
public class TeamLeftAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    boolean bDebug = false;

    private DcMotor armActivator = null;
    private Servo markerArm;

    private BNO055IMU imu;

    double angle;
    double startAngle;
    double targetAngle;

    @Override
    public void runOpMode() {

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

            markerArm = hardwareMap.get(Servo.class, "markerArm");

            armActivator = hardwareMap.get(DcMotor.class, "armActivator");
            armActivator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json";
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        waitForStart();

        runtime.reset();

        while(runtime.seconds() < 3 && opModeIsActive()){ //kill 3 seconds

        }

        runtime.reset();

        while (runtime.seconds() < 0.35 && opModeIsActive()) { //straight

            leftfrontDrive.setPower(.5);
            leftbackDrive.setPower(.5);
            rightfrontDrive.setPower(.5);
            rightbackDrive.setPower(.5);
        }

        turn(90.0); //turn 90 degrees to the left

        runtime.reset();

        while (runtime.seconds() < 1.65) { //Straight

            leftfrontDrive.setPower(.5);
            leftbackDrive.setPower(.5);
            rightfrontDrive.setPower(.5);
            rightbackDrive.setPower(.5);
        }

        turn(32.5); //turn 32.5 degrees to the left

        runtime.reset();

        while (runtime.seconds() < 1.75) { //Backwards

            leftfrontDrive.setPower(-.5);
            leftbackDrive.setPower(-.5);
            rightfrontDrive.setPower(-.5);
            rightbackDrive.setPower(-.5);
        }

        markerArm.setPosition(1);

        runtime.reset();

        while (runtime.seconds() < 3) {

        }

        markerArm.setPosition(0);

        runtime.reset();

        while (runtime.seconds() < 1) {

        }

        runtime.reset();

        while (runtime.seconds() < 2.5) { //straight on the way to the pit

            leftfrontDrive.setPower(.5);
            leftbackDrive.setPower(.5);
            rightfrontDrive.setPower(.5);
            rightbackDrive.setPower(.5);
        }

        while (runtime.seconds() < .5) { //straight over the pit

            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        requestOpModeStop();
    }

    double mod(double a, double b)
    {
        double ret = a % b;
        if (ret < 0)
            ret += b;
        return ret;
    }

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void turn (double turnAngle){
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));

        startAngle = mod(angle, 360.0); //clips range from 0 - 359

        targetAngle = mod((startAngle + turnAngle), 360.0);

        while(opModeIsActive()){

            if(targetAngle - mod(Double.parseDouble(formatAngle(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).angleUnit, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)), 360.0) < 3.0) { //3 degree margin of error
                leftfrontDrive.setPower(0);
                leftbackDrive.setPower(0);
                rightfrontDrive.setPower(0);
                rightbackDrive.setPower(0);
                break;
            }

            leftfrontDrive.setPower(-.3);
            leftbackDrive.setPower(-.3);
            rightfrontDrive.setPower(.3);
            rightbackDrive.setPower(.3);
        }
    }
}
