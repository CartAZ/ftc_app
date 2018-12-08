package org.firstinspires.ftc.teamcode._Test._Sensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode._Libs.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;

import static android.os.SystemClock.sleep;


/**
 * Created by phanau on 1/22/16.
 * Test REV Robotics RevHub IMU
 */

@Autonomous(name="Test: REV IMU Test 1", group ="Test")
//@Disabled
public class RevGyroTestOp extends OpMode {

    private BNO055IMUHeadingSensor mIMU;

    public RevGyroTestOp() {
    }

    public void init() {
        // get hardware IMU and wrap gyro in HeadingSensor object usable below
        mIMU = new BNO055IMUHeadingSensor(hardwareMap.get(BNO055IMU.class, "imu"));
        mIMU.init(7);  // 7: Rev Hub face down with the word Rev facing back
    }

    public void loop() {
        telemetry.addData("orientation: ", mIMU.getOrientation());
        telemetry.addData("position: ", mIMU.getPosition());
    }
 }
