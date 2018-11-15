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

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;
import android.graphics.RectF;
import android.hardware.Camera;
import android.widget.ImageView;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.CameraLib;
import org.firstinspires.ftc.teamcode._Libs.SetPosterizer;
import org.firstinspires.ftc.teamcode._Libs.HeadingSensor;
import org.firstinspires.ftc.teamcode._Libs.RS_Posterize;
import org.firstinspires.ftc.teamcode._Libs.SensorLib;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_FTC2017;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.firstinspires.ftc.teamcode._Libs.CameraLib.colors;

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


// simple data class containing info about image of one column of cryptobox
class ColumnHit {
    int mStart;
    int mEnd;
    public ColumnHit(int start, int end) {
        mStart = start;  mEnd = end;
    }
    public int start() { return mStart; }
    public int end() { return mEnd; }
    public int mid() { return (mStart+mEnd)/2; }
    public int width() { return mEnd-mStart+1; }
}

interface SetBitmap {
    public void setBitmap(Bitmap bm);
}

class DiscretizePosterizer implements CameraLib.Filter {
    public int map(int pix) {
        // return an enumerated color based on the RGB color of the pixel
        int c = colors.eWhite.ordinal();
        if (pix == Color.RED) c = colors.eRed.ordinal();
        else if (pix == Color.GREEN) c = colors.eGreen.ordinal();
        else if (pix == Color.BLUE) c = colors.eBlue.ordinal();
        else if (pix == Color.CYAN) c = colors.eCyan.ordinal();
        else if (pix == Color.YELLOW) c = colors.eYellow.ordinal();
        else if (pix == Color.MAGENTA) c = colors.eMagenta.ordinal();
        return c;
    }
}

// this is a guide step that uses camera image data to
// guide the robot to the indicated bin of the cryptobox
//
class GoToCryptoBoxGuideStep extends AutoLib.MotorGuideStep implements SetMark, SetPosterizer {

    VuforiaLib_FTC2017 mVLib;
    String mVuMarkString;
    OpMode mOpMode;
    int mCBColumn;                      // which Cryptobox column we're looking for
    Pattern mPattern;                   // compiled regexp pattern we'll use to find the pattern we're looking for
    int mColumnOffset;                  // number of columns that have left the left-edge of the frame

    // robot-specific settings - where is your camera relative to the block you're trying to place?
    // if the camera is on the right side of the block, then mCameraOnRight=true and mCameraOffset is positive.
    // if the camera is on the left side of the block, then mCameraOnRight=false and mCameraOffset is negative.
    // if the camera is above the block, then set mCameraOnRight=false and set mCameraOffset to the positive offset of the camera from the left edge of the block.
    final boolean mCameraOnRight = false;     // e.g. orientation of phone on robot (assumed landscape) is with camera on the left
    final float mCameraOffset = -0.5f;        // e.g. camera lens is 0.5 x bin width to the left of the center of the cryptobox column on the block's left side

    ArrayList<ColumnHit> mPrevColumns;  // detected columns on previous pass
    float mPrevBinWidth;                // avg bin width on previous pass

    final int minDoneCount = 5;         // require "done" test to succeed this many consecutive times
    int mDoneCount;

    SetBitmap mSBM;                     // interface through which we tell the controlling opMode about our Bitmap so it can display it
    Bitmap mBmOut;                      // processed bitmap we post to RC phone screen

    RS_Posterize mRsPosterizer;         // RenderScript process used to posterize images

    Paint mPaintRed, mPaintGreen, mPaintBlue;   // used to draw info overlays on image

    AutoLib.MotorGuideStep mMotorGuideStep;     // step used to actually control the motors based on directives from this step and gyro input

    boolean mbLookForBlue;

    public GoToCryptoBoxGuideStep(OpMode opMode, SetBitmap sbm, VuforiaLib_FTC2017 VLib, boolean bLookForBlue, AutoLib.MotorGuideStep motorGuideStep) {
        mOpMode = opMode;
        mCBColumn = 1;     // if we never get a cryptobox directive from Vuforia, go for the first bin
        mbLookForBlue = bLookForBlue;
        mPattern = Pattern.compile(mbLookForBlue ? "^b+" : "^r+");    // look for the given pattern of column colors
        mVLib = VLib;
        mMotorGuideStep = motorGuideStep;
        mPrevColumns = null;
        mColumnOffset = 0;
        mDoneCount = 0;
        mSBM = sbm;
        mPrevBinWidth = 0;

        // create stuff we'll use to draw debug info overlays on the image
        mPaintRed = new Paint();
        mPaintRed.setColor(Color.RED);
        mPaintGreen = new Paint();
        mPaintGreen.setColor(Color.GREEN);
        mPaintBlue = new Paint();
        mPaintBlue.setColor(Color.BLUE);
    }

    public void setMark(String s) {
        mVuMarkString = s;

        // compute index of column that forms the left side of the desired bin.
        // this assumes the camera is mounted to the left of the carried block.
        if (mVuMarkString == "LEFT")
            mCBColumn = 0;
        else
            if (mVuMarkString == "CENTER")
                mCBColumn = 1;
        else
            if (mVuMarkString == "RIGHT")
                mCBColumn = 2;

        // if the camera is on the right side of the block, we want the right edge of the bin.
        if (mCameraOnRight)
            mCBColumn++;
    }

    public void set(ArrayList<AutoLib.SetPower> motorSteps){
        mMotorGuideStep.set(motorSteps);
    }

    public void setPosterizer(RS_Posterize posterizer) {
        mRsPosterizer = posterizer;
    }

    public boolean loop() {
        super.loop();

        mOpMode.telemetry.addData("VuMark", "%s found", mVuMarkString);

        // get most recent frame from camera (through Vuforia)
        RectF rect = new RectF(0,0,1f,0.67f);          // top 2/3 of the image should be enough and avoids floor junk
        Bitmap bmIn = mVLib.getBitmap(rect, 8);                      // get cropped, downsampled image from Vuforia (use 8 for ZTE, 16 for S5)

        //Bitmap bmIn = mVLib.getBitmap(4);                      // get uncropped, downsampled image from Vuforia

        if (bmIn != null) {

            mOpMode.telemetry.addData("image size", "%d x %d", bmIn.getWidth(), bmIn.getHeight());

            // create the output bitmap we'll process and display on the RC phone screen
            mBmOut = Bitmap.createBitmap(bmIn.getWidth(), bmIn.getHeight(), Bitmap.Config.RGB_565);

            // posterize the input bitmap in RenderScript to generate the image we'll analyze and display
            mRsPosterizer.runScript(bmIn, mBmOut);

            // wrap the bitmap in a CameraImage so we can scan it for patterns
            CameraLib.CameraImage frame = new CameraLib.CameraImage(mBmOut);

            // if phone is mounted with the camera on the right, tell CameraImage so it will sample pixels correctly
            frame.setCameraRight(mCameraOnRight);

            // look for cryptobox columns
            // get unfiltered view of colors (hues) by full-image-height column bands
            final int bandSize = 2;         // 1 has better resolution but tends to create multiple hits on one column
            final float minFrac = 0.25f;     // minimum fraction of pixels in band that must be same color to mark it as a color
            String colString = frame.columnRep(bandSize, new DiscretizePosterizer(), null, minFrac);

            // log debug info iff it fits on one line of the DS display ...
            if (colString.length() <= 25)
                mOpMode.telemetry.addData("hue columns", colString);

            // look for occurrences of given pattern of column colors
            ArrayList<ColumnHit> columns = new ArrayList<ColumnHit>(8);       // array of column start/end indices

            for (int i=0; i<colString.length(); i++) {
                // starting at position (i), look for the given pattern in the encoded (rgbcymw) scanline
                Matcher m = mPattern.matcher(colString.substring(i));
                if (m.lookingAt()) {
                    // add start/end info about this hit to the array
                    columns.add(new ColumnHit(i+m.start(), i+m.end()-1));

                    // skip over this match
                    i += m.end();
                }
            }

            // report the matches in telemetry
            for (ColumnHit h : columns) {
                mOpMode.telemetry.addData("found ", "%s from %d to %d", mPattern.pattern(), h.start(), h.end());
            }

            // add annotations to the bitmap showing detected column centers
            Canvas canvas = new Canvas(mBmOut);
            for (ColumnHit h : columns) {
                canvas.drawLine(h.mid()*bandSize,0, h.mid()*bandSize, 30, mbLookForBlue? mPaintRed : mPaintBlue);
                canvas.drawLine(h.start()*bandSize,30, h.end()*bandSize, 30, mbLookForBlue? mPaintRed : mPaintBlue);
            }

            int nCol = columns.size();

            // compute average distance between columns = distance between outermost / #bins --
            // if we can only see 1 column, use ratio of bin-width (~7.5") to column-width (~1.5"); otherwise use prev estimate
            float avgBinWidth = nCol>1 ? (float)(columns.get(nCol-1).end() - columns.get(0).start()) / (float)(nCol-1) :
                                nCol==1 ? columns.get(0).width() * 5.0f : mPrevBinWidth;
            mPrevBinWidth = avgBinWidth;

            // try to handle case where a column has left (or entered) the left-edge of the frame between the prev view and this one
            if (mPrevColumns != null  &&  mPrevColumns.size()>0  &&  nCol>0  && avgBinWidth > 0) {

                // if the left-most column of the previous frame started at the left edge of the frame
                // and the left edge of the current left-most column is about a bin-width to the right of the right edge of the
                // left-most column of the previous frame
                // then it's probably the case that the current left-most column is actually the second column of the previous frame.
                if (mPrevColumns.get(0).start() == 0 && columns.get(0).start() > (mPrevColumns.get(0).end()+0.5*avgBinWidth)) {
                    mColumnOffset++;
                }

                // if the left-most column of the previous frame was not near the left edge of the frame
                // but now there is a column at the left edge, then one probably entered the frame.
                if (mColumnOffset > 0 && mPrevColumns.get(0).start() > 0.5*avgBinWidth  &&  columns.get(0).start() == 0) {
                    mColumnOffset--;
                }
            }

            mOpMode.telemetry.addData("data", "avgWidth= %f  mColOff=%d", avgBinWidth, mColumnOffset);

            final float tanCameraHalfFOV = 28.0f/50.0f;       // horizontal half angle FOV of S5 camera is atan(28/50) or about 29.25 degrees

            // estimate distance from the cryptoboxes
            float normBW = avgBinWidth*2 / colString.length();                   // avg bin width in normalized camera space (-1 .. +1)
            float distance = -1;                                            // distance to cryptobox in inches --- -1 means "don't know"
            if (normBW > 0){
                distance = 7.5f / (normBW * tanCameraHalfFOV);                    // distance to bins given they are 7.5" wide;
                mOpMode.telemetry.addData("distance (in)", distance);
            }

            // show target point on image
            canvas.drawLine(mBmOut.getWidth()/2-5, 15, mBmOut.getWidth()/2+5, 15, mPaintGreen);

            // if we found some columns, try to correct course using their positions in the image
            boolean bTargetBefore = mCBColumn < mColumnOffset;
            boolean bTargetAfter = mCBColumn-mColumnOffset >= nCol;
            if (!bTargetBefore && !bTargetAfter) {
                // the target column is in the view ...

                // compute camera offset from near-side column of target bin (whichever side camera is to the block holder)
                float cameraBinOffset = avgBinWidth * mCameraOffset;
                // camera target is center of target column + camera offset in image-string space
                float cameraTarget = columns.get(mCBColumn-mColumnOffset).mid() + cameraBinOffset;

                // show target point if it's in the current image
                float x = cameraTarget*bandSize;
                if (x>=0 && x<mBmOut.getWidth())
                    canvas.drawLine(x, 10, x, 20, mPaintGreen);

                // the above computed target point should be in the middle of the image if we're on course -
                // if not, correct our course to center it --
                // compute fractional error = fraction of image offset of target from center = [-1 .. +1]
                float error = (cameraTarget - (float)colString.length()/2.0f) / ((float)colString.length()/2.0f);

                // compute motor correction from error through PID --
                // for now, convert image-string error to angle --
                // negate it because image coordinates are positive to the right but angles are positive to the left (CCW)
                float angError = -(float)(Math.atan(error * tanCameraHalfFOV) * 180.0/Math.PI);
                mOpMode.telemetry.addData("data", "target=%f  error=%f angError=%f", cameraTarget, error, angError);

                // tell the subsidiary motor guide step which way to steer -- the heading (orientation) will either be
                // constant (squirrely wheels) or change to match the direction, depending on the motor guide step we're given.
                ((AutoLib.SetDirectionHeadingPower)mMotorGuideStep).setRelativeDirection(angError);
            }
            else
                // the target column is not in the view -- if it's now to the "before" or "after" side of the frame
                // (i.e. when scanning left to right, to the left or right of the frame), turn that way ...
                // normally this should not happen because we constantly steer for the target, but if
                // steering is suppressed for some reason (like avoiding obstacles), it may.
            if (bTargetBefore){
                // e.g. move/turn hard left ... (depending on squirrely or normal drive)
                ((AutoLib.SetDirectionHeadingPower)mMotorGuideStep).setRelativeDirection(90.0f);
            }
            else
            if (bTargetAfter){
                // e.g. move/turn hard right ...(depending on squirrely or normal drive)
                ((AutoLib.SetDirectionHeadingPower)mMotorGuideStep).setRelativeDirection(-90.0f);
            }

            // when we're really close ...
            if (distance > 0  &&  distance < 12) {          // for now, stop at 12" from bins
                // require completion test to pass some min number of times in a row to believe it
                mDoneCount++;
                if (mDoneCount >= minDoneCount) {
                    // return "done" -- assume the next step will stop motors if needed
                    return true;
                }
            }
            else
                mDoneCount = 0;         // reset the "done" counter

            mOpMode.telemetry.addData("data", "doneCount=%d", mDoneCount);

            // tell the calling opMode about the bitmap so it can display it
            mSBM.setBitmap(mBmOut);

            // save column hits for next pass to help handle columns leaving the field of view of
            // the camera as we get close.
            mPrevColumns = columns;

        }
        else
            mOpMode.telemetry.addData("getBitmap()", "no new frame");


        return false;  // haven't found anything yet
    }

    public void stop() {
    }
}


//@Autonomous(name="FirstRelicRecAuto1", group ="Auto")
//@Disabled
public class FirstRelicRecAuto1 extends OpMode implements SetBitmap {

    boolean bDone;
    AutoLib.Sequence mSequence;             // the root of the sequence tree
    DcMotor mMotors[];                      // motors, some of which can be null: assumed order is fr, br, fl, bl
    GyroSensor mGyro;                       // gyro to use for heading information
    SensorLib.CorrectedGyro mCorrGyro;      // gyro corrector object
    VuforiaLib_FTC2017 mVLib;               // Vuforia wrapper object used by Steps
    RS_Posterize mRsPosterize;              // RenderScript posterization filter

    ImageView mView;
    Bitmap mBitmap; //the bitmap you will display
    //private static final Object bmLock = new Object(); //synchronization lock so we don't display and write

    AutoLib.MotorGuideStep mGuideStep;      // member variable so that init() can set it and start() can use it

    double mTime;   // time of last loop() call -- used to compute frames per second

    public void setBitmap(Bitmap b)
    {
        mBitmap = b;
    }

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
        // make a step that will steer the robot given guidance from the GoToCryptoBoxGuideStep
        // this can be a gyro-stabilized squirrely wheel steering step or just a debug logging step
        AutoLib.MotorGuideStep motorGuideStep = debug ?
                    new AutoLib.MotorLogStep(this) :
                    new AutoLib.SquirrelyGyroGuideStep(this, 0, 0, mCorrGyro, null, null, 0.6f);
        // make a step that guides the motion step by looking for a particular (red or blue) Cryptobox
        // it also implements the SetMark interface so VuforiaGetMarkStep can call it to tell which box to go for
        // and in this case, is given a SquirrelyGyroGuide step to give steering commands to.
        mGuideStep  = new GoToCryptoBoxGuideStep(
                this, this, mVLib, bLookForBlue, motorGuideStep);
        // make and add to the sequence the step that looks for the Vuforia marker and sets the column (Left,Center,Right)
        // the motion terminator step should look for
        mSequence.add(new VuforiaGetMarkStep(this, mVLib, (SetMark)mGuideStep));
        // make and add the Step that goes to the indicated Cryptobox bin
        mSequence.add(new AutoLib.GuidedTerminatedDriveStep(this, mGuideStep, null, mMotors));
        // make and add a step that tells us we're done
        mSequence.add(new AutoLib.LogTimeStep(this,"Done!", 5));

        mView = (ImageView)((Activity)hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.OpenCVOverlay);
        mView.post(new Runnable() {
            @Override
            public void run() {
                mView.setAlpha(1.0f);
            }
        });
        mBitmap = null;
    }

    @Override public void start()
    {
        // start out not-done
        bDone = false;

        // start Vuforia scanning
        mVLib.start();

        // create a RenderScript posterizer -- we do it here because it takes too long to do in init()
        mRsPosterize = new RS_Posterize();
        mRsPosterize.createScript(this.hardwareMap.appContext);

        // tell the camera-based guide step about the RenderScript posterizer it should use
        ((SetPosterizer)mGuideStep).setPosterizer(mRsPosterize);

        // get start time for frames/sec computation
        mTime = getRuntime();
    }

    @Override
    public void loop() {
        // log frames per second
        double dTime = getRuntime() - mTime;
        if (dTime > 0)
            telemetry.addData("frames/sec", 1.0/dTime);
        mTime = getRuntime();       // for next time ...

        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            telemetry.addData("sequence finished", "");

        //display!
        mView.post(new Runnable() {
            @Override
            public void run() {
                //synchronized (bmLock) {
                    mView.setImageBitmap(mBitmap);
                    mView.invalidate();
                //}
            }
        });
    }

    @Override
    public void stop() {
        super.stop();
        mVLib.stop();     // make sure the Camera is released by Vuforia
    }

}

