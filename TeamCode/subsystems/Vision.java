/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.ArrayList;
import java.util.Arrays;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcOpenCvDetector;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcCommonLib.ftclib.FtcEocvColorBlobProcessor;
import TrcCommonLib.ftclib.FtcOpMode;
import TrcCommonLib.ftclib.FtcRawEocvColorBlobPipeline;
import TrcCommonLib.ftclib.FtcRawEocvVision;
import TrcCommonLib.ftclib.FtcVision;
import TrcCommonLib.ftclib.FtcVisionAprilTag;
import TrcCommonLib.ftclib.FtcVisionEocvColorBlob;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;

/**
 * This class implements AprilTag/TensorFlow/Eocv Vision for the game season. It creates and initializes all the
 * vision target info as well as providing info for the robot, camera and the field. It also provides methods to get
 * the location of the robot and detected targets.
 */
public class Vision
{
    private static final String moduleName = Vision.class.getSimpleName();

    public enum SampleType
    {
        BlueSample,
        BlueSpecimen,
        RedSample,
        RedSpecimen,
        YellowSample,
        AnySample
    }   //enum PixelType

    // Warning: EOCV converts camera stream to RGBA whereas Desktop OpenCV converts it to BGRA. Therefore, the correct
    // color conversion must be RGBA (or RGB) to whatever color space you want to convert.
    //
    // YCrCb Color Space.
    private static final int colorConversion = Imgproc.COLOR_RGB2YCrCb;
    private static final double[] yellowSampleColorThresholds = {150.0, 250.0, 110.0, 160.0, 20.0, 100.0};
    private static final double[] whitePixelColorThresholds = {250.0, 255.0, 100.0, 130.0, 120.0, 140.0};
    private static final double[] redBlobColorThresholds = {20.0, 120.0, 180.0, 240.0, 90.0, 120.0};
    private static final double[] blueBlobColorThresholds = {20.0, 250.0, 40.0, 250.0, 160.0, 240.0};

    private static final TrcOpenCvColorBlobPipeline.FilterContourParams pixelFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(1000.0)
            .setMinPerimeter(120.0)
            .setWidthRange(50.0, 1000.0)
            .setHeightRange(10.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.2, 5.0);
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams blobFilterContourParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(1000.0)
            .setMinPerimeter(100.0)
            .setWidthRange(20.0, 1000.0)
            .setHeightRange(20.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.8, 1.25);

    private final TrcDbgTrace tracer;
    private final Robot robot;
    private final WebcamName webcam1, webcam2;
    private FtcRawEocvColorBlobPipeline rawColorBlobPipeline;
    public FtcRawEocvVision rawColorBlobVision;
    public FtcVisionAprilTag aprilTagVision;
    private AprilTagProcessor aprilTagProcessor;
    public FtcVisionEocvColorBlob yellowSampleVision;
    private FtcEocvColorBlobProcessor yellowSampleProcessor;
    public FtcVisionEocvColorBlob redBlobVision;
    private FtcEocvColorBlobProcessor redBlobProcessor;
    public FtcVisionEocvColorBlob blueBlobVision;
    private FtcEocvColorBlobProcessor blueBlobProcessor;
    public FtcVision vision;
    private int lastTeamPropPos = 0;

    /**
     * Constructor: Create an instance of the object.@param robot specifies the robot object.
     */
    public Vision(Robot robot)
    {
        FtcOpMode opMode = FtcOpMode.getInstance();

        this.tracer = new TrcDbgTrace(moduleName);
        this.robot = robot;
        this.webcam1 = opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM1);
        this.webcam2 = RobotParams.Preferences.hasWebCam2?
            opMode.hardwareMap.get(WebcamName.class, RobotParams.HWNAME_WEBCAM2): null;
        if (RobotParams.Preferences.tuneColorBlobVision)
        {
            OpenCvCamera openCvCam;

            if (RobotParams.Preferences.showVisionView)
            {
                int cameraViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
                openCvCam = OpenCvCameraFactory.getInstance().createWebcam(webcam1, cameraViewId);
                openCvCam.showFpsMeterOnViewport(false);
            }
            else
            {
                openCvCam = OpenCvCameraFactory.getInstance().createWebcam(webcam1);
            }

            tracer.traceInfo(moduleName, "Starting RawEocvColorBlobVision...");
            rawColorBlobPipeline = new FtcRawEocvColorBlobPipeline(
                "rawColorBlobPipeline", colorConversion, whitePixelColorThresholds, pixelFilterContourParams, true);
            // By default, display original Mat.
            rawColorBlobPipeline.setVideoOutput(0);
            rawColorBlobPipeline.setAnnotateEnabled(true);
            rawColorBlobVision = new FtcRawEocvVision(
                "rawColorBlobVision", RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT, null, null,
                openCvCam, RobotParams.CAM_ORIENTATION);
            setRawColorBlobVisionEnabled(false);
        }
        else
        {
            // Creating Vision Processors for VisionPortal.
            ArrayList<VisionProcessor> visionProcessorsList = new ArrayList<>();

            if (RobotParams.Preferences.useAprilTagVision)
            {
                tracer.traceInfo(moduleName, "Starting AprilTagVision...");
                FtcVisionAprilTag.Parameters aprilTagParams = new FtcVisionAprilTag.Parameters()
                    .setDrawTagIdEnabled(true)
                    .setDrawTagOutlineEnabled(true)
                    .setDrawAxesEnabled(false)
                    .setDrawCubeProjectionEnabled(false)
//                    .setLensIntrinsics(
//                        RobotParams.WEBCAM_FX, RobotParams.WEBCAM_FY, RobotParams.WEBCAM_CX, RobotParams.WEBCAM_CY)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
                aprilTagVision = new FtcVisionAprilTag(aprilTagParams, AprilTagProcessor.TagFamily.TAG_36h11);
                aprilTagProcessor = aprilTagVision.getVisionProcessor();
                visionProcessorsList.add(aprilTagProcessor);
            }

            if (RobotParams.Preferences.useColorBlobVision)
            {
                tracer.traceInfo(moduleName, "Starting ColorBlobVision...");
            }

                       VisionProcessor[] visionProcessors = new VisionProcessor[visionProcessorsList.size()];
            visionProcessorsList.toArray(visionProcessors);
            if (RobotParams.Preferences.useWebCam)
            {
                // Use USB webcams.
                vision = new FtcVision(
                    webcam1, webcam2, RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT,
                    RobotParams.Preferences.showVisionView, visionProcessors);
            }
            else
            {
                // Use phone camera.
                vision = new FtcVision(
                    RobotParams.Preferences.useBuiltinCamBack?
                        BuiltinCameraDirection.BACK: BuiltinCameraDirection.FRONT,
                    RobotParams.CAM_IMAGE_WIDTH, RobotParams.CAM_IMAGE_HEIGHT,
                    RobotParams.Preferences.showVisionView, visionProcessors);
            }
            // Disable all vision until they are needed.
            setRawColorBlobVisionEnabled(false);
            for (VisionProcessor processor: visionProcessors)
            {
                vision.setProcessorEnabled(processor, false);
            }
        }
    }   //Vision

    /**
     * This method returns the front webcam.@return front webcam.
     */
    public WebcamName getFrontWebcam()
    {
        return webcam1;
    }   //getFrontWebcam

    /**
     * This method returns the rear webcam.@return rear webcam.
     */
    public WebcamName getRearWebcam()
    {
        return webcam2;
    }   //getRearWebcam

    /**
     * This method returns the active camera if we have two webcams.@return active camera.
     */
    public WebcamName getActiveWebcam()
    {
        return vision.getActiveWebcam();
    }   //getActiveWebcam

    /**
     * This method sets the active webcam.@param webcam specifies the webcam to be set as active.
     */
    public void setActiveWebcam(WebcamName webcam)
    {
        vision.setActiveWebcam(webcam);
    }   //setActiveWebcam

    /**
     * This method displays the exposure settings on the dashboard. This helps tuning camera exposure.
     * @param lineNum specifies the dashboard line number to display the info.
     */
    public void displayExposureSettings(int lineNum)
    {
        long[] exposureSetting = vision.getExposureSetting();
        long currExposure = vision.getCurrentExposure();
        int[] gainSetting = vision.getGainSetting();
        int currGain = vision.getCurrentGain();

        robot.dashboard.displayPrintf(
            lineNum, "Exp: %d (%d:%d), Gain: %d (%d:%d)",
            currExposure, exposureSetting[0], exposureSetting[1],
            currGain, gainSetting != null? gainSetting[0]: 0, gainSetting != null? gainSetting[1]: 0);
    }   //displayExposureSettings

    /**
     * This method returns the color threshold values of rawColorBlobVision.
     * @return array of color threshold values.
     */
    public double[] getRawColorBlobThresholds()
    {
        return rawColorBlobPipeline != null? rawColorBlobPipeline.getColorThresholds(): null;
    }   //getRawColorBlobThresholds

    /**
     * This method sets the color threshold values of rawColorBlobVision.
     * @param colorThresholds specifies an array of color threshold values.
     */
    public void setRawColorBlobThresholds(double... colorThresholds)
    {
        if (rawColorBlobPipeline != null)
        {
            rawColorBlobPipeline.setColorThresholds(colorThresholds);
        }
    }   //setRawColorBlobThresholds


    /**
     * This method enables/disables raw ColorBlob vision.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setRawColorBlobVisionEnabled(boolean enabled)
    {
        if (rawColorBlobVision != null)
        {
            rawColorBlobVision.setPipeline(enabled? rawColorBlobPipeline: null);
        }
    }   //setRawColorBlobVisionEnabled

    /**
     * This method checks if raw ColorBlob vision is enabled.
     * @return true if enabled, false if disabled.
     */
    public boolean isRawColorBlobVisionEnabled()
    {
        return rawColorBlobVision != null && rawColorBlobVision.getPipeline() != null;
    }   //isRawColorBlobVisionEnabled

    /**
     * This method calls RawColorBlob vision to detect the color blob for color threshold tuning.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected raw color blob object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getDetectedRawColorBlob(int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> colorBlobInfo =
            rawColorBlobVision != null? rawColorBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0): null;

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(lineNum, "ColorBlob: %s", colorBlobInfo != null? colorBlobInfo: "Not found.");
        }

        return colorBlobInfo;
    }   //getDetectedRawColorBlob

    /**
     * This method enables/disables AprilTag vision.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setAprilTagVisionEnabled(boolean enabled)
    {
        if (aprilTagProcessor != null)
        {
            vision.setProcessorEnabled(aprilTagProcessor, enabled);
        }
    }   //setAprilTagVisionEnabled

    /**
     * This method checks if AprilTag vision is enabled.
     *
     * @return true if enabled, false if disabled.
     */
    public boolean isAprilTagVisionEnabled()
    {
        return aprilTagProcessor != null && vision.isVisionProcessorEnabled(aprilTagProcessor);
    }   //isAprilTagVisionEnabled

    /**
     * This method calls AprilTag vision to detect the AprilTag object.
     *
     * @param id specifies the AprilTag ID to look for, null if match to any ID.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected AprilTag object info.
     */
    public TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> getDetectedAprilTag(Integer id, int lineNum)
    {
        TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo =
            aprilTagVision.getBestDetectedTargetInfo(id, null);

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(lineNum, "AprilTag: %s", aprilTagInfo != null? aprilTagInfo: "Not found.");
        }

        return aprilTagInfo;
    }   //getDetectedAprilTag

    /**
     * This method calculates the robot's absolute field location with the detected AprilTagInfo.
     *
     * @param aprilTagInfo specifies the detected AprilTag info.
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose(TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo)
    {
        TrcPose2D robotPose = null;

        if (aprilTagInfo != null)
        {
            TrcPose2D aprilTagPose = RobotParams.APRILTAG_POSES[aprilTagInfo.detectedObj.aprilTagDetection.id - 1];
            TrcPose2D cameraPose = aprilTagPose.subtractRelativePose(aprilTagInfo.objPose.toPose2D());
            robotPose = cameraPose.subtractRelativePose(RobotParams.BACKCAM_POSE);
            tracer.traceInfo(
                moduleName,
                "AprilTagId=" + aprilTagInfo.detectedObj.aprilTagDetection.id +
                ", aprilTagFieldPose=" + aprilTagPose +
                ", aprilTagPoseFromCamera=" + aprilTagInfo.objPose.toPose2D() +
                ", cameraPose=" + cameraPose +
                ", robotPose=%s" + robotPose);
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method uses vision to find an AprilTag and uses the AprilTag's absolute field location and its relative
     * position from the camera to calculate the robot's absolute field location.
     *
     * @return robot field location.
     */
    public TrcPose2D getRobotFieldPose()
    {
        TrcPose2D robotPose = null;

        if (aprilTagVision != null)
        {
            // Find any AprilTag in view.
            TrcVisionTargetInfo<FtcVisionAprilTag.DetectedObject> aprilTagInfo = getDetectedAprilTag(null, -1);

            if (aprilTagInfo != null)
            {
                robotPose = getRobotFieldPose(aprilTagInfo);
            }
        }

        return robotPose;
    }   //getRobotFieldPose

    /**
     * This method enables/disables vision for the specified pixel type.
     * @param sampleType specifies the pixel type to be detected.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setSampleVisionEnabled(SampleType sampleType, boolean enabled)
    {
        switch (sampleType)
        {
            case YellowSample:
                if (yellowSampleProcessor != null)
                {
                    vision.setProcessorEnabled(yellowSampleProcessor, enabled);
                }
                break;
            case RedSample:
                if (redBlobProcessor != null)
                {
                    vision.setProcessorEnabled(redBlobProcessor, enabled);
                }
                break;
            case BlueSample:
                if (blueBlobProcessor != null)
                {
                    vision.setProcessorEnabled(blueBlobProcessor, enabled);
                }
                break;
        }
    }   //setSampleVisionEnabled

    /**
     * This method checks if vision is enabled for the specified pixel type.
     * @param sampleType specifies the pixel type to be detected.
     * @return true if enabled, false if disabled.
     */
    public boolean isPixelVisionEnabled(SampleType sampleType)
    {
        boolean enabled = false;

        switch (sampleType)
        {
            case RedSample:
                enabled = redBlobProcessor != null && vision.isVisionProcessorEnabled(redBlobProcessor);
                break;

            case BlueSample:
                enabled = blueBlobProcessor != null && vision.isVisionProcessorEnabled(blueBlobProcessor);
                break;

            case YellowSample:
                enabled = yellowSampleProcessor != null && vision.isVisionProcessorEnabled(yellowSampleProcessor);
                break;

            case AnySample:
                enabled = redBlobProcessor != null && vision.isVisionProcessorEnabled(redBlobProcessor) ||
                          blueBlobProcessor != null && vision.isVisionProcessorEnabled(blueBlobProcessor) ||
                          yellowSampleProcessor != null && vision.isVisionProcessorEnabled(yellowSampleProcessor);
                break;
        }

        return enabled;
    }   //isPixelVisionEnabled

    /**
     * This method calls ColorBlob vision to detect the specified Pixel object.
     * @param sampleType specifies the pixel type to be detected.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return detected Purple Pixel object info.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedPixel(
        SampleType sampleType, int lineNum)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> pixelInfo = null;
        String pixelName = null;

        switch (sampleType)
        {
            case YellowSample:
                pixelInfo = yellowSampleVision != null? yellowSampleVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                break;

            case AnySample:
                ArrayList<TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> pixelList = new ArrayList<>();

                pixelInfo = yellowSampleVision != null? yellowSampleVision.getBestDetectedTargetInfo(
                    this::validatePixel, this::compareDistance, 0.0, 0.0): null;
                if (pixelInfo != null)
                {
                    pixelList.add(pixelInfo);
                }

                TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>[] pixels =
                    new TrcVisionTargetInfo[pixelList.size()];
                pixelList.toArray(pixels);
                if (pixels.length > 1)
                {
                    Arrays.sort(pixels, this::compareDistance);
                }
                pixelInfo = pixels[0];
                pixelName = pixelInfo.detectedObj.label;
                break;
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "%s: %s", pixelName, pixelInfo != null? pixelInfo: "Not found.");
        }

        return pixelInfo;
    }   //getDetectedPixel

    /**
     * This method enables/disables RedBlob vision. @param enabled specifies true to enable, false to disable.
     */
    public void setRedBlobVisionEnabled(boolean enabled)
    {
        if (redBlobProcessor != null)
        {
            vision.setProcessorEnabled(redBlobProcessor, enabled);
        }
    }   //setRedBlobVisionEnabled

    /**
     * This method checks if RedBlob vision is enabled.@return true if enabled, false if disabled.
     */
    public boolean isRedBlobVisionEnabled()
    {
        return redBlobProcessor != null && vision.isVisionProcessorEnabled(redBlobProcessor);
    }   //isRedBlobVisionEnabled

    /**
     * This method enables/disables BlueBlob vision.@param enabled specifies true to enable, false to disable.
     */
    public void setBlueBlobVisionEnabled(boolean enabled)
    {
        if (blueBlobProcessor != null)
        {
            vision.setProcessorEnabled(blueBlobProcessor, enabled);
        }
    }   //setBlueBlobVisionEnabled

    /**
     * This method checks if BlueBlob vision is enabled. @return true if enabled, false if disabled.
     */
    public boolean isBlueBlobVisionEnabled()
    {
        return blueBlobProcessor != null && vision.isVisionProcessorEnabled(blueBlobProcessor);
    }   //isBlueBlobVisionEnabled

    /**
     * This method detects the team prop and determine its position.
     * @param alliance specifies the alliance color to look for team prop.
     * @param lineNum specifies the dashboard line number to display the detected object info, -1 to disable printing.
     * @return team prop position 1, 2, or 3, 0 if not found.
     */
    public int getDetectedTeamPropPosition(FtcAuto.Alliance alliance, int lineNum)
    {
        int pos = 0;
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> teamPropInfo = null;

        if (alliance == FtcAuto.Alliance.RED_ALLIANCE)
        {
            if (redBlobVision != null)
            {
                teamPropInfo = redBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0);
            }
        }
        else
        {
            if (blueBlobVision != null)
            {
                teamPropInfo = blueBlobVision.getBestDetectedTargetInfo(null, null, 0.0, 0.0);
            }
        }

        if (teamPropInfo != null)
        {
            double teamPropXPos = teamPropInfo.rect.x + teamPropInfo.rect.width/2.0;
            double oneThirdScreenWidth = RobotParams.CAM_IMAGE_WIDTH/3.0;
            String ledLabel = null;

            if (teamPropXPos < oneThirdScreenWidth)
            {
                pos = 1;
            }
            else if (teamPropXPos < oneThirdScreenWidth*2)
            {
                pos = 2;
            }
            else
            {
                pos = 3;
            }
        }

        if (lineNum != -1)
        {
            robot.dashboard.displayPrintf(
                lineNum, "%s: %s (pos=%d)",
                alliance == FtcAuto.Alliance.RED_ALLIANCE? "RedBlob": "BlueBlob",
                teamPropInfo != null? teamPropInfo: "Not found", pos);
        }

        if (pos != 0)
        {
            lastTeamPropPos = pos;
        }

        return pos;
    }   //getDetectedTeamPropPosition

    /**
     * This method returns the last detected team prop position.
     * @return last team prop position, 0 if never detected it.
     */
    public int getLastDetectedTeamPropPosition()
    {
        return lastTeamPropPos;
    }   //getLastDetectedTeamPropPosition

    /**
     * This method enables/disables TensorFlow vision.
     * @param enabled specifies true to enable, false to disable.
     */
    public void setTensorFlowVisionEnabled(boolean enabled)
    {
       /* if (tensorFlowProcessor != null)
        {
            vision.setProcessorEnabled(tensorFlowProcessor, enabled);
        }*/
    }   //setTensorFlowVisionEnabled

     /**
     * This method validates the detected pixel is really a pixel by checking its physical width to be about 3 inches.
     * @param pixelInfo specifies the detected pixel info object.
     * @return true if it passes the test, false otherwise.
     */
    public boolean validatePixel(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> pixelInfo)
    {
        // Pixel is 3-inch wide.
        return pixelInfo.objWidth > 2.0 && pixelInfo.objWidth < 4.0;
    }   //validatePixel

    /**
     * This method is called by the Arrays.sort to sort the target object by increasing distance.
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has closer distance than b, 0 if a and b have equal distances, positive value
     *         if a has higher distance than b.
     */
    private int compareDistance(
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> a,
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> b)
    {
        return (int)((b.objPose.y - a.objPose.y)*100);
    }   //compareDistance

    /**
     * This method is called by the Arrays.sort to sort the target object by decreasing confidence.
     * @param a specifies the first target
     * @param b specifies the second target.
     * @return negative value if a has higher confidence than b, 0 if a and b have equal confidence, positive value
     *         if a has lower confidence than b.
     */
}   //class Vision
