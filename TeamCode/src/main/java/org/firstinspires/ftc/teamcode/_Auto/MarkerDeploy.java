package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MarkerDeploy", group="Autonomous")
public class MarkerDeploy extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private Servo markerArm;
    private DcMotor armActivator = null;

    boolean bDebug = false;

    @Override
    public void init() {
        try {
            armActivator = hardwareMap.get(DcMotor.class, "armActivator");
            armActivator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armActivator.setDirection(DcMotor.Direction.REVERSE);

            markerArm = hardwareMap.get(Servo.class, "markerArm");
            markerArm.setPosition(0);
        } catch (IllegalArgumentException iax) {
            bDebug = true;
        }

        runtime.reset();

    }


    public void loop () {
        while (runtime.seconds() < 1) {
            armActivator.setPower(1);
        }


        while (runtime.seconds() < 2) {
            markerArm.setPosition(1);
        }
        while (runtime.seconds() < 2.5){
            markerArm.setPosition(0);
        }


        while (runtime.seconds() < 3) {
            armActivator.setPower(-1);
        }
    }
}
