/**
 * Created by nanok on 11/19/2018.
 */

package org.firstinspires.ftc.teamcode._Auto;

import android.view.ViewDebug;

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
import org.firstinspires.ftc.teamcode._Libs.AutoLib;

import java.util.Locale;

@Autonomous(name="encoderLeftAuto", group="Autonomous")
public class encoderLeftAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    AutoLib.Sequence mSequence;     // the root of the sequence tree

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
        //this code runs after the init button is pressed
        AutoLib.HardwareFactory mf = null;

        try {
            leftfrontDrive = mf.getDcMotor("frontLeft");
            leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightfrontDrive = mf.getDcMotor("frontRight");
            rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);

            leftbackDrive = mf.getDcMotor("backLeft");
            leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightbackDrive = mf.getDcMotor("backRight");
            rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightbackDrive.setDirection(DcMotor.Direction.REVERSE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");

            armActivator = hardwareMap.get(DcMotor.class, "armActivator");
            armActivator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            mSequence = new AutoLib.LinearSequence();

        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double power = 0.8; // shaft encoder at 28 ppr * 60:1 gearbox
        boolean stop = true;

        waitForStart(); //the rest of the code begins after the play button is pressed

        sleep(3000);

        mSequence.add(new AutoLib.EncoderMotorStep(leftfrontDrive, power, 100, stop));
        mSequence.add(new AutoLib.EncoderMotorStep(rightfrontDrive, power, 100, stop));
        mSequence.add(new AutoLib.EncoderMotorStep(leftbackDrive, power, 100, stop));
        mSequence.add(new AutoLib.EncoderMotorStep(rightbackDrive, power, 100, stop));

        requestOpModeStop(); //end of autonomous
    }

    double mod(double a, double b){
        if (a < 0) {
            a += b;
        }
        else if(a > b){
            a -= b;
        }
        return a;
    }

    String formatAngle (AngleUnit angleUnit, double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void drive(double time, double power){
        runtime.reset();
        while(runtime.seconds() < time){
            leftfrontDrive.setPower(power);
            leftbackDrive.setPower(power);
            rightfrontDrive.setPower(power);
            rightbackDrive.setPower(power);
        }
    }

    public void turn (double turnAngle){
        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angle = Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle)); //turns the angle from the imu which is a string into a double

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
