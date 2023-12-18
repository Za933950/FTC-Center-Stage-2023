/*
 * Copyright (c) 2021 OpenFTC Team
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.*;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class ColorAutonomousDetection extends LinearOpMode {

    OpenCvWebcam webcam1 = null;

    private static DcMotor leftFront,leftBack,rightFront,rightBack, slideMotor;

    private class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;

        double leftavgfin;
        double rightavgfin;


        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0, 0.0, 0.0);


        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");


            Rect leftRect = new Rect(1, 1, 319, 359);
            Rect rightRect = new Rect(320, 1, 319, 359);


            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 2);
            Imgproc.rectangle(outPut, rightRect, rectColor, 2);


            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);


            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop,rightCrop,2);


            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);


            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];



            if (leftavgfin > rightavgfin) {

                telemetry.addLine("right");

            } else if (rightavgfin > leftavgfin) {

                telemetry.addLine("left");
            }



            telemetry.addData("rightfinavg", rightavgfin);
            telemetry.addData("leftfinavg", leftavgfin);
            telemetry.addData("rightfin", rightavg);
            telemetry.addData("leftfin", leftavg);
            telemetry.update();



            return (outPut);




        }



    }

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

            webcam1.setPipeline(new examplePipeline());

            webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                public void onOpened() {
                    webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
                }

                public void onError(int errorCode) {

                }
            });


            leftFront = hardwareMap.dcMotor.get("frontLeft");
            leftBack = hardwareMap.dcMotor.get("backLeft");
            rightFront = hardwareMap.dcMotor.get("frontRight");
            rightBack = hardwareMap.dcMotor.get("backRight");
        }



    }


}




