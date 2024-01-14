package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.RedConeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.pipelines.RedConeDetectionPipeline;




@Autonomous(name="RedClose", group="Robot")
@Config

public class RedClose extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal, leftBack, leftFront,rightFront, rightBack,slideMotor;

    private Servo autoDrop;

    private Servo dropperServo, planeServo, rotateServo;




    String verticalLeftEncoderName = "frontLeft";
    String verticalRightEncoderName = "frontRight";
    String horizontalEncoderName = "intakeMotor";

    OpenCvWebcam webcam;
    public static double XPOSITION_1 = 0;
    public static double YPOSITION_1 = 0;
    public static double HEADING_1 = 0;
    public static double STOP = 13;
    public static double XPOSITION_2 = 0;
    public static double YPOSITION_2 = 0;
    public static double HEADING_2 = 0;
    public static double XPOSITION_3 = 0;
    public static double YPOSITION_3 = 0;
    public static double HEADING_3 = 0;

    private double ENCODER_TICKS_PER_ROTATION = 1120 * (2.0/3);





    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");
        autoDrop = hardwareMap.servo.get("autoDrop");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        dropperServo = hardwareMap.servo.get("dropper");
        planeServo = hardwareMap.servo.get("plane");
        rotateServo = hardwareMap.servo.get("rotate");



        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal);
        Thread positionUpdate = new Thread(odometry);

        positionUpdate.start();



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

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(.75);


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
        }

        autoDrop.setPosition(0.625);


        waitForStart();


        if (opModeIsActive()) {

            //Left
            if (location == 0){
                GoToPosition.goToPosition(-2.5, -11, -27, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
                GoToPosition.goToPosition(6.7, -24.7, -27, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
                autoDrop.setPosition(.4);
                GoToPosition.goToPosition(-23, -1.7, 0, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .675);
                GoToPosition.goToPosition(-21, -60, -90, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
            }
            //Center
            if (location == 1){
                GoToPosition.goToPosition(-6, -30, 0, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
                autoDrop.setPosition(.4);
                GoToPosition.goToPosition(-23, -1.7, 0, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
                GoToPosition.goToPosition(-21, -60, -90, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
            }
            //Right
            if (location == 2){
                GoToPosition.goToPosition(-14, -27, 0, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
                autoDrop.setPosition(.4);
                GoToPosition.goToPosition(-22, -1.7, 0, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
                GoToPosition.goToPosition(-25, -60, 0, 11, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .75);
            }
            slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1150));
            sleep(5000);
            slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1200));


        }

    }
    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
    }

}

