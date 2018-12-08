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
import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;

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

    private BNO055IMUHeadingSensor mIMU;

    float targetAngle;

    @Override
    public void runOpMode() {
        //this code runs after the init button is pressed
        try {
            leftfrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            leftfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightfrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            rightfrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftbackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            leftbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftbackDrive.setDirection(DcMotor.Direction.REVERSE);

            rightbackDrive = hardwareMap.get(DcMotor.class, "backRight");
            rightbackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");

            armActivator = hardwareMap.get(DcMotor.class, "armActivator");
            armActivator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(7);  // 7: Rev Hub face down with the word Rev facing back

        waitForStart(); //the rest of the code begins after the play button is pressed

        sleep(3000);

        drive(0.35,0.5);

        turn(90.0f); //turn 90 degrees to the left

        drive(1.45,0.5);

        turn(40.0f); //turn 35 degrees to the left

        drive(2.5,-0.5);

        sleep(1000);

        markerArm.setPosition(1);

        sleep(1000);

        markerArm.setPosition(0);

        sleep(1000);

        drive(4.0,1.0);

        requestOpModeStop(); //end of autonomous
    }

    float mod(float a, float b){
        if (a < 0) {
            a += b;
        }
        else if(a > b){
            a -= b;
        }
        return a;
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

    public void driveEncoder(){
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfrontDrive.setTargetPosition(1120);

        leftfrontDrive.setPower(0.5);
        rightfrontDrive.setPower(0.5);
        leftbackDrive.setPower(0.5);
        rightbackDrive.setPower(0.5);

        while(leftfrontDrive.isBusy() && opModeIsActive()) {

        }

        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        leftbackDrive.setPower(0);
        leftbackDrive.setPower(0);
    }

    public void turn (float turnAngle){ //left turn

        targetAngle = mIMU.getHeading() + turnAngle;

        while(opModeIsActive()){

            if(targetAngle - mIMU.getHeading() < 3.0) { //3 degree margin of error
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
