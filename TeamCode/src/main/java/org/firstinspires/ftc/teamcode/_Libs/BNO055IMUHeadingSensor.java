package org.firstinspires.ftc.teamcode._Libs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import static android.os.SystemClock.sleep;

// wrapper for BNO055IMU gyro that implements our HeadingSensor interface
public class BNO055IMUHeadingSensor implements HeadingSensor {
    BNO055IMU mIMU;
    float mHeadingOffset = 0;

    public BNO055IMUHeadingSensor(BNO055IMU imu) {
        mIMU = imu;
    }

    public float getHeading() {
        // return sensor heading value plus initial offset (heading at sensor zero) wrapped to -180..+180
        return SensorLib.Utils.wrapAngle(mIMU.getAngularOrientation().firstAngle + mHeadingOffset);
    }

    public boolean haveHeading() { return true; }

    public void setHeadingOffset(float offset) { mHeadingOffset = offset; }

    public float getPitch() {
        return mIMU.getAngularOrientation().thirdAngle;
    }

    public float getRoll() {
        return mIMU.getAngularOrientation().secondAngle;
    }

    public Orientation getOrientation() {
        return mIMU.getAngularOrientation();
    }

    public Position getPosition() {
        return mIMU.getPosition();
    }

    public void init(int orientation) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";

        mIMU.initialize(parameters);

        // this may take too long to do in init -- if so, move to start() or loop() first time
        if (orientation >= 0)
            setRevOrientation(mIMU, orientation);
    }

    // this function sets CONFIG and SIGN bytes for REV mounted in various orientations.
    // the default orientation of the IMU axes in the REV hub is like this:
    //       __________________________
    //      |                          |
    //      |  REV                     |
    //      |           Z@------ Y     |
    //      |            |             |        ---------> front of vehicle
    //      |            |             |
    //      |            X             |
    //      |__________________________|

    // we will use the convention PITCH = +X, ROLL = +Y, YAW(Heading) = +Z
    // which corresponds to mounting the REV hub laying flat on the vehicle as shown above.

    private void setRevOrientation(BNO055IMU imu, int orientation) {
        // other orientations need these settings:
        byte[] config = { 0x24, 0x24, 0x6, 0x9, 0x21, 0x24, 0x6, 0x21};
        byte[] sign   = { 0x0,  0x6,  0x1, 0x3, 0x4,  0x6,  0x2, 0x7};
        // 0: flat with top of REV label on right side of vehicle (default):
        // byte AXIS_MAP_CONFIG_BYTE = 0x24;    // Z=Z Y=Y X=X
        // byte AXIS_MAP_SIGN_BYTE = 0x0;       // X Y Z
        // 1: flat with R in REV closest to front of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x24;    // Z=Z Y=-Y X=-X
        // byte AXIS_MAP_SIGN_BYTE = 0x6;       // -X -Y Z
        // 2: upright longitudinally with R nearest the front of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x6;     // Z=-X Y=-Y X=-Z
        // byte AXIS_MAP_SIGN_BYTE = 0x1;       // X Y -Z     ?? if 0x7 -X -Y -Z, X and Y are LH rotations
        // 3: upright crosswise with REV facing forward:
        // byte AXIS_MAP_CONFIG_BYTE = 0x9;     // Z=-X Y=Z X=-Y
        // byte AXIS_MAP_SIGN_BYTE = 0x3;       // X -Y -Z    ?? if 0x5 -X Y -Z, X and Y are LH rotations
        // 4: flat with top of REV label along back side of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x21;    // Z=Z Y=-X X=Y
        // byte AXIS_MAP_SIGN_BYTE = 0x4;       // -X Y Z
        // 5: flat with top of REV label on left side of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x24;    // Z=Z Y=-Y X=-X
        // byte AXIS_MAP_SIGN_BYTE = 0x6;       // -X -Y Z
        // 6: upright longitudinally with V nearest the front of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x6;     // Z=-X Y=Y X=Z
        // byte AXIS_MAP_SIGN_BYTE = 0x2;       // X -Y Z     ?? if 0x4 -X Y Z, X and Y are LH rotations
        // 7: flat face down with top of REV label along back side of vehicle:
        // byte AXIS_MAP_CONFIG_BYTE = 0x21;    // Z=-Z Y=X X=Y
        // byte AXIS_MAP_SIGN_BYTE = 0x7;       // -X -Y -Z


        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal);
        sleep(19); //Changing to CONFIG mode require a delay of 19ms before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, config[orientation]);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, sign[orientation]);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal);
        sleep(7); //Changing back to any operating mode requires a delay of 7ms
    }
}
