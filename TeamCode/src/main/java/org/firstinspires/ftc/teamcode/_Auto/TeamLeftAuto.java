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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@Autonomous(name="TeamLeftAuto", group="Autonomous")
public class TeamLeftAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    boolean bDebug = false;

    private DcMotor armActivator = null;
    private Servo markerArm;

    private BNO055IMU imu;

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

            markerArm = hardwareMap.get(Servo.class, "markerArm");
            markerArm.setPosition(0);

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
    }

    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop() {

        final Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = Integer.parseInt(formatAngle(angles.angleUnit, angles.firstAngle));
        double startAngle;
        double targetAngle;

        runtime.reset();

        while(runtime.seconds() < 3){ //kill 3 seconds

        }

        runtime.reset();

        while (runtime.seconds() < 0.50) { //straight

            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        runtime.reset();

        startAngle = angle % 360; //clips range from 0 -360
        if (startAngle > 270) {
            targetAngle = (startAngle + 87) % 360;
            while (angle % 360 <= 357) { //turn 90 degrees to the left +- 3 degrees error

                leftfrontDrive.setPower(-.5);
                leftbackDrive.setPower(-.5);
                rightfrontDrive.setPower(.5);
                rightbackDrive.setPower(.5);
            }
            while (angle % 360 <= targetAngle) { //turn 90 degrees to the left +- 3 degrees error

                leftfrontDrive.setPower(-.5);
                leftbackDrive.setPower(-.5);
                rightfrontDrive.setPower(.5);
                rightbackDrive.setPower(.5);
            }
        }
        else{
            while ((angle % 360) - startAngle <= 87) { //turn 90 degrees to the left +- 3 degrees error

                leftfrontDrive.setPower(-.5);
                leftbackDrive.setPower(-.5);
                rightfrontDrive.setPower(.5);
                rightbackDrive.setPower(.5);
            }
        }

        runtime.reset();

        while (runtime.seconds() < 1.5) { //Straight

            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        startAngle = angle % 360; //clips range from 0 -360
        if (startAngle > 315) {
            targetAngle = (startAngle + 42) % 360;
            while (angle % 360 <= 357) { //turn 45 degrees to the left +- 3 degrees error

                leftfrontDrive.setPower(-.5);
                leftbackDrive.setPower(-.5);
                rightfrontDrive.setPower(.5);
                rightbackDrive.setPower(.5);
            }
            while (angle % 360 <= targetAngle) { //turn 45 degrees to the left +- 3 degrees error

                leftfrontDrive.setPower(-.5);
                leftbackDrive.setPower(-.5);
                rightfrontDrive.setPower(.5);
                rightbackDrive.setPower(.5);
            }
        }
        else{
            while ((angle % 360) - startAngle <= 42) { //turn 45 degrees to the left +- 3 degrees error

                leftfrontDrive.setPower(-.5);
                leftbackDrive.setPower(-.5);
                rightfrontDrive.setPower(.5);
                rightbackDrive.setPower(5.);
            }
        }

        runtime.reset();

        while (runtime.seconds() < 2) { //Backwards

            leftfrontDrive.setPower(-1);
            leftbackDrive.setPower(-1);
            rightfrontDrive.setPower(-1);
            rightbackDrive.setPower(-1);
        }

        markerArm.setPosition(1);

        runtime.reset();

        while (runtime.seconds() < 1) {

        }

        markerArm.setPosition(0);

        runtime.reset();

        while (runtime.seconds() < 1) { //straight

            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        telemetry.addLine()
                .addData("status", imu.getSystemStatus().toShortString())
                .addData("calib", imu.getCalibrationStatus().toString())
                .addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
    }

    @Override
    public void stop () {

    }

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
