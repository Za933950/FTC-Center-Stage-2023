package PipeLines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class BlueConeDetectionPipeline extends OpenCvPipeline {
    boolean viewportPaused;
    Mat mask = new Mat();
    Mat output = new Mat();

    Point topLeft = new Point(0, 0);
    Point bottomleft = new Point(639, 359);
    Point topRight = new Point(639, 0);
    Point bottomRight = new Point(1279, 359);

    Mat leftTopHalf = new Mat();
    Mat rightTopHalf = new Mat();
    Scalar avgRight;
    Scalar avgLeft;
    Telemetry telemetry;
    int location;
    double blueleftVal;
    double blueRightVal;

    public Scalar lowerBound = new Scalar(0, 0, 85);
    public Scalar upperBound = new Scalar(60, 60, 255);

    public BlueConeDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2RGB);
        Imgproc.blur(input, input, new Size(10,10));
        Core.inRange(input, lowerBound, upperBound, mask);
        output.release();
        Core.bitwise_and(input, input, output, mask);
        Imgproc.rectangle(
                output,
                new Point(
                        0,
                        input.rows() / 2),
                new Point(
                        input.cols(),
                        input.rows()),
                new Scalar(0, 0, 0), Imgproc.FILLED);
        leftTopHalf = output.submat(new Rect(topLeft, bottomleft));
        rightTopHalf = output.submat(new Rect(topRight, bottomRight));
        avgLeft = Core.mean(leftTopHalf);
        avgRight = Core.mean(rightTopHalf);
        blueleftVal = avgLeft.val[2];
        blueRightVal = avgRight.val[2];
        if (blueleftVal > 5) {
            location = 0;
            telemetry.addData("Left",0);
            //left
        } else if (blueRightVal > 2) {
            location = 1;
            telemetry.addData("Center",1);
            //center
        } else {
            location = 2;
            telemetry.addData("Right",2);
            //right
        }

        telemetry.addData("avgLeft", avgLeft);
        telemetry.addData("avgRight", avgRight);
        telemetry.addData("location", location);
        telemetry.update();

        return output;


    }

    public int getLocation() {
        blueleftVal = avgLeft.val[2];
        blueRightVal = avgRight.val[2];
        if (blueleftVal > 5) {
            location = 0;
            telemetry.addData("Left",0);
            //left
        } else if (blueRightVal > 2) {
            location = 1;
            telemetry.addData("Center",1);
            //center
        } else {
            location = 2;
            telemetry.addData("Right",2);
            //right
        }

        return location;
    }
}

