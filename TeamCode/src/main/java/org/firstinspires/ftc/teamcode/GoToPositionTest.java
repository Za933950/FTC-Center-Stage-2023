package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvWebcam;




@Autonomous(name="Test Test", group="Robot")
@Config

public class GoToPositionTest extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal, leftBack, leftFront,rightFront, rightBack;

    private Servo autoDrop;


    String verticalLeftEncoderName = "frontLeft";
    String verticalRightEncoderName = "frontRight";
    String horizontalEncoderName = "intakeMotor";

    OpenCvWebcam webcam;
    public static double XPOSITION_1 = 0;
    public static double YPOSITION_1 = 0;
    public static double HEADING_1 = 0;
    public static double STOP = 10;
    public static double XPOSITION_2 = 0;
    public static double YPOSITION_2 = 0;
    public static double HEADING_2 = 0;
    public static double XPOSITION_3 = -5;
    public static double YPOSITION_3 = -27;
    public static double HEADING_3 = 0;





    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
        autoDrop = hardwareMap.servo.get("autoDrop");



        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal);
        Thread positionUpdate = new Thread(odometry);

        positionUpdate.start();

        /*

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        RedConeDetectionPipeline samplePipeline = new RedConeDetectionPipeline(telemetry);
        webcam.setPipeline(samplePipeline);


        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        int location = 0;


        while (!opModeIsActive()) {
            location = samplePipeline.getLocation();
            if (location == 0) {
                telemetry.addData("Left",0);
            } if (location == 1) {
                telemetry.addData("Center", 1);
            } if (location == 2) {
                telemetry.addData("Right", 2);
            }
            telemetry.update();
        }*/

        autoDrop.setPosition(0.625);


        waitForStart();


        if (opModeIsActive()) {

                /*GoToPosition.goToPosition(0, -46,0,15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .8);
                GoToPosition.goToPosition(0, -46,190,15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .8);*/
            GoToPosition.goToPosition(XPOSITION_1, YPOSITION_1, HEADING_1, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
            GoToPosition.goToPosition(XPOSITION_2, YPOSITION_2, HEADING_2, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
            autoDrop.setPosition(.4);
            GoToPosition.goToPosition(XPOSITION_3, YPOSITION_3, HEADING_3, STOP, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);


            /*GoToPosition.goToPosition(24, 48,90,15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, 1);
            GoToPosition.goToPosition(0,0,0, 15, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, 1);
            sleep(5000);*/

        }

    }

}

