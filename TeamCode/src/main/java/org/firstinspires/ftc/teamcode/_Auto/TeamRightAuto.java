package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TeamRightAuto", group="Autonomous")
public class TeamRightAuto extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;

    boolean bDebug = false;

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

            markerArm = hardwareMap.get(Servo.class, "markerArm");
            markerArm.setPosition(0);
        }
        catch (IllegalArgumentException iax) {
            bDebug = true;
        }
    }

    @Override
    public void loop(){


        //Kill 3 seconds

        runtime.reset();

        while(runtime.seconds() < 3){

        }

        runtime.reset();

        while(runtime.seconds() < 0.5){

            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        runtime.reset();

        while(runtime.seconds() < 0.5){

            leftfrontDrive.setPower(-1);
            leftbackDrive.setPower(-1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        runtime.reset();

        while(runtime.seconds() < 1.5){

            leftfrontDrive.setPower(1);
            leftbackDrive.setPower(1);
            rightfrontDrive.setPower(1);
            rightbackDrive.setPower(1);
        }

        runtime.reset();

        markerArm.setPosition(1);

        while(runtime.seconds() < 1){

        }

        markerArm.setPosition(0);

        runtime.reset();

        while(runtime.seconds() < 3){
            leftfrontDrive.setPower(-1);
            leftbackDrive.setPower(-1);
            rightfrontDrive.setPower(-1);
            rightbackDrive.setPower(-1);
        }

        runtime.reset();

    }

    @Override
    public void stop(){

    }
}
