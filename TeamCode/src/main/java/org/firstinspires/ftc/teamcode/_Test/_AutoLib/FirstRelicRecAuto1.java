/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode._Test._AutoLib;

import android.graphics.Bitmap;
import android.graphics.Point;
import android.graphics.Rect;
import android.hardware.Camera;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.DistanceSensor;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This OpMode uses a Step that uses the VuforiaLib_FTC2017 library to determine
 * which column of the shelves to fill first, then
 * moves the robot under gyro control while using the camera to look for the
 * correct Cryptobox to stop at.
 */

// define an interface through which a Step (or anything else) can be told the
// identity of the Vuforia target that we should use
interface SetMark {
    public void setMark(String s);
}

class VuforiaGetMarkStep extends AutoLib.Step {

    VuforiaLib_FTC2017 mVLib;
    OpMode mOpMode;
    SetMark mSMStep;

    public VuforiaGetMarkStep(OpMode opMode, VuforiaLib_FTC2017 VLib, SetMark step) {
        mOpMode = opMode;
        mVLib = VLib;
        mSMStep = step;
    }

    public boolean loop() {
        super.loop();
        mVLib.loop();       // update recognition info
        RelicRecoveryVuMark vuMark = mVLib.getVuMark();
        boolean found = (vuMark != RelicRecoveryVuMark.UNKNOWN);
        if (found) {
            // Found an instance of the template -- tell "MoveTo.. step which one
            mSMStep.setMark(vuMark.toString());
        }
        return found;       // done?
    }
}

class BlueFilter implements CameraLib.Filter {
    public int map(int hue) {
        // map 4 (cyan) to 5 (blue)
        if (hue == 4)
            return 5;
        else
            return hue;
    }
}

// this is a guide step that starts up the Camera and uses it to
// determine when the appropriate CryptoBox is in sight, at which time
// it terminates the GuidedTerminatedDriveStep of which it is part
//
class LookForCryptoBoxStep extends AutoLib.Step implements SetMark {
    String mVuMarkString;
    VuforiaLib_FTC2017 mVLib;
    boolean mCameraActive;
    OpMode mOpMode;
    int mCBColumn;              // which Cryptobox column we're looking for
    Pattern mPattern;           // compiled regexp pattern we'll use to find the pattern we're looking for
    CameraLib.Filter mBlueFilter;       // filter to map cyan to blue

    public LookForCryptoBoxStep(OpMode opMode, VuforiaLib_FTC2017 VLib, String pattern) {
        mOpMode = opMode;
        mCBColumn = -1;     // unknown
        mPattern = Pattern.compile(pattern);    // look for the given pattern of column colors
        mBlueFilter = new BlueFilter();
        mVLib = VLib;
    }

    public void setMark(String s) { mVuMarkString = s; }

    public boolean loop() {
        super.loop();
        mOpMode.telemetry.addData("VuMark", "%s found", mVuMarkString);

        // get most recent frame from camera (through Vuforia)
        Bitmap bitmap = mVLib.getBitmap(4);                         // get reduced-resolution image from Vuforia
        CameraLib.CameraImage frame = new CameraLib.CameraImage(bitmap);    // .. and wrap it in a CameraImage

        if (bitmap != null && frame != null) {
            // look for cryptobox columns
            // get unfiltered view of colors (hues) by full-image-height column bands
            final int bandSize = 8;
            String colHue = frame.columnHue(bandSize);

            // log debug info ...
            mOpMode.telemetry.addData("hue columns", colHue);

            // look for occurrences of given pattern of column colors and report them in telemetry
            int patternStart = 0;
            int patternSize = 0;
            for (int i=0; i<colHue.length(); i++) {
                // starting at position (i), look for the given pattern in the encoded (rgbcymw) scanline
                Matcher m = mPattern.matcher(colHue.substring(i));
                if (m.lookingAt() /* && m.groupCount() == 1 */) {
                    mOpMode.telemetry.addData("found ", "%s from %d to %d", mPattern.pattern(), i+m.start(), i+m.end()-1);
                    i += m.end();       // skip over this match
                }
            }
            
        }

        return false;  // haven't found anything yet
    }

    public void stop() {
    }
}


//@Autonomous(name="FirstRelicRecAuto1", group ="Auto")
//@Disabled
public class FirstRelicRecAuto1 extends OpMode {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    GyroSensor mGyro;                       // gyro to use for heading information
    SensorLib.CorrectedGyro mCorrGyro;      // gyro corrector object
    LookForCryptoBoxStep mTerminatorStep;   // needs to be class data so stop() function can access camera
    VuforiaLib_FTC2017 mVLib;               // Vuforia wrapper object used by Steps

    public void init() {}

    public void init(boolean bLookForBlue)
    {
        // get the hardware
        AutoLib.HardwareFactory mf = null;
        final boolean debug = true;
        if (debug)
            mf = new AutoLib.TestHardwareFactory(this);
        else
            mf = new AutoLib.RealHardwareFactory(this);

        // get the motors: depending on the factory we created above, these may be
        // either dummy motors that just log data or real ones that drive the hardware
        // assumed order is fr, br, fl, bl
        mMotors = new DcMotor[4];
        mMotors[0] = mf.getDcMotor("fr");
        mMotors[1] = mf.getDcMotor("br");
        (mMotors[2] = mf.getDcMotor("fl")).setDirection(DcMotor.Direction.REVERSE);
        (mMotors[3] = mf.getDcMotor("bl")).setDirection(DcMotor.Direction.REVERSE);

        // get hardware gyro
        mGyro = mf.getGyro("gyro");

        // wrap gyro in an object that calibrates it and corrects its output
        mCorrGyro = new SensorLib.CorrectedGyro(mGyro);
        mCorrGyro.calibrate();

        // best to do this now, which is called from opmode's init() function
        mVLib = new VuforiaLib_FTC2017();
        mVLib.init(this, null);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();
        // make a step that terminates the motion step by looking for a particular (red or blue) Cryptobox
        mTerminatorStep = new LookForCryptoBoxStep(this, mVLib, bLookForBlue ? "^b+" : "^r+");
        // make and add to the sequence the step that looks for the Vuforia marker and sets the column (Left,Center,Right)
        // the motion terminator step should look for
        mSequence.add(new VuforiaGetMarkStep(this, mVLib, mTerminatorStep));
        AutoLib.MotorGuideStep guideStep = new AutoLib.SquirrelyGyroGuideStep(this, 90, 0, mCorrGyro, null, null, 0.5f);
        // make and add the Step that goes to the indicated Cryptobox bin
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, guideStep, mTerminatorStep, mMotors));
        // make and add a step that stops all motors
        mSequence.add(new AutoLib.MoveByTimeStep(mMotors, 0, 0, true));
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();
    }

    @Override
    public void loop() {

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");
    }

    @Override
    public void stop() {
        super.stop();
        mVLib.stop();     // make sure the Camera is released by Vuforia
    }

}

