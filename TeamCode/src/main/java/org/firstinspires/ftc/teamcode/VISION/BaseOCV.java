package org.firstinspires.ftc.teamcode.VISION;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import lombok.Getter;

public class BaseOCV extends OpMode {
    OpenCvWebcam webcam;
    SamplePipeline pipeline;

    @Override
    public void init() {
        // Initialize webcam (USB webcam on Control Hub)
        int camViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1"); // match config name
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, camViewId);

        // Create and set our pipeline
        pipeline = new SamplePipeline();
        webcam.setPipeline(pipeline);

        // Open camera asynchronously and start streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Note: use a modest resolution for speed (e.g. 640x480 or 320x240)
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void loop() {
        // Display distance and angle computed by the pipeline
        telemetry.addData("Distance (cm)", pipeline.getDistance());
        telemetry.addData("Angle (deg)", pipeline.getAngle());
        telemetry.update();
    }

    // Custom pipeline class
    static class SamplePipeline extends OpenCvPipeline {
        Mat homographyMat;
        Mat warpMat = new Mat();
        Mat hsv = new Mat();
        Mat thresh = new Mat();
        @Getter
        double distance = 0;
        @Getter
        double angle = 0;

        // Placeholder: known object width (in cm) and focal length (pixels) for distance calc
        double KNOWN_WIDTH = 5.0;      // e.g. sample actual width
        double FOCAL_LENGTH = 500.0;   // calibrate this (see below)

        @Override
        public void init(Mat firstFrame) {
            // Define four source points (on the floor) and destination (top-down view)
            // TODO: replace these with real calibration points for your setup
            MatOfPoint2f srcPts = new MatOfPoint2f(
                    new Point(100, 200),  // top-left corner in camera frame
                    new Point(500, 200),  // top-right
                    new Point(100, 400),  // bottom-left
                    new Point(500, 400)   // bottom-right
            );
            MatOfPoint2f dstPts = new MatOfPoint2f(
                    new Point(0, 0),          // map to bird's-eye rectangle
                    new Point(400, 0),
                    new Point(0, 300),
                    new Point(400, 300)
            );
            homographyMat = Imgproc.getPerspectiveTransform(srcPts, dstPts);
        }

        @Override
        public Mat processFrame(Mat input) {
            // 1. Warp perspective to get top-down (bird's-eye) view of the field floor
            Imgproc.warpPerspective(input, warpMat, homographyMat, new Size(400, 300));

            // 2. Convert to HSV (or YCrCb) and threshold to isolate target color
            Imgproc.cvtColor(warpMat, hsv, Imgproc.COLOR_RGB2HSV);
            // Example: threshold for a green-ish object (tune values)
            Scalar lower = new Scalar(30, 100, 100);
            Scalar upper = new Scalar(90, 255, 255);
            Core.inRange(hsv, lower, upper, thresh);

            // 3. Find contours on the thresholded image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Pick the largest contour by area
                MatOfPoint largest = Collections.max(contours, Comparator.comparingDouble(Imgproc::contourArea));

                // Compute a rotated bounding box around the contour
                MatOfPoint2f largest2f = new MatOfPoint2f(largest.toArray());
                RotatedRect box = Imgproc.minAreaRect(largest2f);

                // Distance estimation: use known width and focal length
                double pixelWidth = Math.max(box.size.width, box.size.height);
                distance = (KNOWN_WIDTH * FOCAL_LENGTH) / pixelWidth;  // D = (W * F) / P:contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}

                // Orientation: angle from vertical or horizontal of box, or using center offset
                angle = -box.angle;  // RotatedRect.angle gives tilt (negative sign may align conventions)
            }

            // IMPORTANT: release Mats if created inside processFrame (to avoid memory leak)
            // (In this simple example, we reuse mats; ensure no Mat is left un-released.)

            return warpMat; // return warped image for display (could also return original)
        }
    }
}