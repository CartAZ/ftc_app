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
package org.firstinspires.ftc.teamcode._Libs;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.RectF;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;

/**
 * This library encapsulates the code needed by OpModes using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 *
 * Vuforia uses the phone's camera to inspect its surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * This library assumes a field configuration with four targets, one under each of four beacons,
 * mounted two each on two adjacent walls of the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * IMPORTANT: In order to use this Library, you need to obtain your own Vuforia license key as
 * is explained below.
 */

// we need to break into the "impl" class to get access to its close() function so we can
// really shut it down and get back control of the camera
class myVuforiaLocalizerImpl2 extends VuforiaLocalizerImpl
{
    public myVuforiaLocalizerImpl2(VuforiaLocalizer.Parameters parameters)
    {
        super(parameters);
    }
    public void _close()  { close(); }
}

public class VuforiaLib_3D implements HeadingSensor, LocationSensor {

    myVuforiaLocalizerImpl2 vuforia;
    VuforiaTrackables mTrackables = null;
    OpenGLMatrix mLastLocation = null;
    VuforiaTrackable mLastVisible = null;

    OpMode mOpMode;

    BlockingQueue<VuforiaLocalizer.CloseableFrame> mFrameQueue;
    VuforiaLocalizer.CloseableFrame mCF;

    public void init(OpMode opMode, String licenseKey, String database, String names[]) {

        // remember this so we can do telemetry output
        mOpMode = opMode;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
                (licenseKey != null && licenseKey.length() > 0) ? licenseKey :
                "ARf809H/////AAAAGRswBQwUCUJ5nqfgZxGbDEQ8oO7YP5GdnbReYr8ZHinqQ74OsP7UdOxNZJDmhaF2OeGD20jpSexpr2CcXGSGuHXNB2p9Z6zUNLDTfEggL+yg4ujefoqdkSpCqZf1medpwh3KXcK76FcfSJuqEudik2PC6kQW/cqJXnnHofVrrDTzJmWMnK3hlqTMjig81DEPMAHbRnA5wn7Eu0irnmqqboWyOlQ0xTF+P4LVuxaOUFlQC8zPqkr1Gvzvix45paWtyuLCnS9YDWMvI1jIM4giMrTRCT0lG8F+vkuKMiK647KJp9QIsFdWQ0ecQhau3ODNQ03pcTzprVN72b9VObpv6FNBpjGKRAcA59xlZiM2l6fc";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = new myVuforiaLocalizerImpl2(parameters); // ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track. These particular data
         * sets are stored in the 'assets' part of our application (you'll see them in the Android
         * Studio 'Project' view over there on the left of the screen).
         */
        mTrackables = this.vuforia.loadTrackablesFromAsset(database);
        int i = 0;
        for (VuforiaTrackable t : mTrackables) {
            mTrackables.get(i).setName(names[i]);
            if (i < names.length-1)
                i++;
        }

        // assign an arbitrary field transformation to all Trackables so we can get position info relative to them
        for (VuforiaTrackable t : mTrackables) {
            OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                    .translation(0, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            t.setLocation(wheelsTargetLocationOnField);
        }

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone with the back camera facing the front of the robot (see our
         * choice of BACK camera above) and in landscape mode with the USB jack either up or down.
         * The starting alignment between the robot's and phone's axes is assumed to be phone
         * lying on the robot in portrait mode with the long axis (Y) pointing at the front of the robot.
         *
         * Rotations are right handed: i.e. positive rotations wrap each axis into the next (X-Y-Z-X)
         * in the direction of your fingers when your right thumb points down the remaining axis.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        // AngleUnit.DEGREES, 90, -90, 0));                // USB on top
                        AngleUnit.DEGREES, -90, 90, 0));       // USB on bottom

        /**
         * Let the trackable listeners we care about know where the phone is. We know that each
         * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
         * we have not ourselves installed a listener of a different type.
         */
        for (VuforiaTrackable t : mTrackables) {
            ((VuforiaTrackableDefaultListener)t.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // get access to video frames so we can do other processing like looking for red/blue beacons
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB format for the image
        vuforia.setFrameQueueCapacity(1);
        mFrameQueue = vuforia.getFrameQueue();
        mCF = null;

    }

    public void start()
    {
        /** Start tracking the data sets we care about. */
        mTrackables.activate();
    }

    public void loop()
    {
        mLastLocation = null;    // reset each time so we can tell if we currently have any target visible

        for (VuforiaTrackable t : mTrackables) {

            if (((VuforiaTrackableDefaultListener)t.getListener()).isVisible())
                mLastVisible = t;

            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)t.getListener()).getRobotLocation();
            if (robotLocationTransform != null) {
                mLastLocation = robotLocationTransform;
            }
        }
    }


    public void stop()
    {
        /** Stop tracking the data sets we care about. */
        mTrackables.deactivate();

        // close down Vuforia NOW so other code can use the camera
        vuforia._close();
    }

    // return name of last detected Trackable
    public String getName() {
        return mLastVisible != null ? mLastVisible.getName() : "none";
    }

    // return lastLocation matrix (may be null)
    public OpenGLMatrix getLastLocation()
    {
        return mLastLocation;
    }

    public VectorF getFieldPosition()
    {
        if (mLastLocation != null)
            return mLastLocation.getTranslation();
        else
            return null;
    }

    public Orientation getOrientation(AxesReference ref, AxesOrder order, AngleUnit unit)
    {
        if (mLastLocation != null)
            return Orientation.getOrientation(mLastLocation, ref, order, unit);
        else
            return null;
    }

    public Orientation getOrientation()
    {
        return this.getOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public float getPitch()
    {
        return this.getOrientation().firstAngle;
    }

    public float getRoll()
    {
        return this.getOrientation().secondAngle;
    }

    public float getYaw()
    {
        return this.getOrientation().thirdAngle;    // -180 .. 0 .. +180
    }

    // implements HeadingSensor interface
    public float getHeading()
    {
       return getYaw();
    }
    public boolean haveHeading()
    {
        return (mLastLocation != null);
    }
    public void setHeadingOffset(float offset) {}   // not used; Vuforia headings are field-absolute

    // implements LocationSensor interface
    public VectorF getLocation() { return getFieldPosition(); }
    public boolean haveLocation() { return (mLastLocation != null);}       // is there valid location data?

    /**
     * Some simple utilities that extract information from a transformation matrix
     * and format it in a form palatable to a human being.
     * For sanity's sake, display translations in inches rather than mm.
     */

    static public String formatPosition(OpenGLMatrix transformationMatrix) {
        //return transformationMatrix.formatAsTransform();
        VectorF translation = transformationMatrix.getTranslation();
        translation.multiply(1.0f/25.4f);       // convert from mm to inches
        return String.format("%s inches", translation.toString());
    }

    static public String formatOrientation(OpenGLMatrix transformationMatrix) {
        //return transformationMatrix.formatAsTransform();
        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return String.format("%s", orientation.toString());
    }

    // get access to video frames for other uses
    public VuforiaLocalizer.CloseableFrame getFrame()
    {
        if (mFrameQueue != null)
            try {
                mCF = mFrameQueue.take();
            }
            catch (Exception e) {
                mCF = null;
            }
        return mCF;
    }

    public void releaseFrame()
    {
        if (mCF != null)
            mCF.close();
        mCF = null;         // forget the frame
    }

    public Bitmap getBitmap(int sample) {
        return getBitmap(new RectF(0,0,1,1), sample);
    }

    public Bitmap getBitmap(RectF rect, int sample) {
        try {
            VuforiaLocalizer.CloseableFrame frame = mFrameQueue.take();
            int img = 0;
            for (; img < frame.getNumImages(); img++) {
                //telemetry.addData("Image format " + img, frame.getImage(img).getFormat());
                if (frame.getImage(img).getFormat() == PIXEL_FORMAT.RGB565) break;
            }

            if (img == frame.getNumImages()) throw new IllegalArgumentException("Incorrect format");

            // get the Image with the correct format and extract its data
            Image image = frame.getImage(img);
            ByteBuffer byteBuffer = image.getPixels();

            int h = image.getHeight();    // height of src data
            int w = image.getWidth();     // width of src data

            // convert the data to a Bitmap
            byteBuffer.rewind();
            Bitmap bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(byteBuffer);

            // crop the Bitmap if requested
            Bitmap bitmapCropped = bitmap;
            int ch = Math.round(h*rect.height());    // height of cropped src data
            int cw = Math.round(w*rect.width());     // width of cropped src data
            if (ch < h || cw < w) {
                bitmapCropped = Bitmap.createBitmap(bitmap,
                        Math.max(0, Math.round(w*rect.left)), Math.max(0, Math.round(h*rect.top)), cw, ch);
                h = ch; w = cw;
            }

            // scale the (possibly cropped) result bitmap by the "sample" factor
            Bitmap bitmapScaled = Bitmap.createScaledBitmap(bitmapCropped, w/sample, h/sample, true);

            frame.close();

            return bitmapScaled;
        }
        catch (Exception e) {}

        return null;
    }
}
