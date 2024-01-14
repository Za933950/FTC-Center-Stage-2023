package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.BlueConeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.pipelines.BlueConeDetectionPipeline;


@Autonomous(name="BlueClose", group="Robot")
@Config

public class BlueClose extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal, leftBack, leftFront,rightFront, rightBack,slideMotor;

    private Servo autoDrop;
    private Servo dropperServo, planeServo, rotateServo;


    String verticalLeftEncoderName = "frontLeft";
    String verticalRightEncoderName = "frontRight";
    String horizontalEncoderName = "intakeMotor";
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    IMU imu;

    OpenCvWebcam webcam;
    public static double XPOSITION_1 = -5;
    public static double YPOSITION_1 = -27;
    public static double HEADING_1 = 0;
    public static double STOP = 15;
    public static double XPOSITION_2 = -5;
    public static double YPOSITION_2 = -27;
    public static double HEADING_2 = 0;
    public static double XPOSITION_3 = -5;
    public static double YPOSITION_3 = -27;
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
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal, imu);
        Thread positionUpdate = new Thread(odometry);

        positionUpdate.start();

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(.75);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BlueConeDetectionPipeline samplePipeline = new BlueConeDetectionPipeline(telemetry);
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
        }

        autoDrop.setPosition(0.625);


        waitForStart();


        if (opModeIsActive()) {
                //Left
                if (location == 0){
                    GoToPosition.goToPosition(4, -27, 0, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    autoDrop.setPosition(.4);
                    GoToPosition.goToPosition(0, -17, 0, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    GoToPosition.goToPosition(37.25, -20, -90, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1000));
                }
                //Center
                if (location == 1){
                    GoToPosition.goToPosition(-4, -30, 0, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    autoDrop.setPosition(.4);
                    GoToPosition.goToPosition(0, -17, 0, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    GoToPosition.goToPosition(37.25, -27, -90, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1150));

                }
                //Right
                if (location == 2){

                    GoToPosition.goToPosition(0, -29, 90, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    GoToPosition.goToPosition(-4.6, -29, 90, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    autoDrop.setPosition(.4);
                    GoToPosition.goToPosition(37.25, -31, -90, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, .6);
                    slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1150));

                }
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            slideMotor.setTargetPosition(convertDegreesToEncoderTicks(980));
            while (Math.abs(slideMotor.getCurrentPosition() - convertDegreesToEncoderTicks(980)) > convertDegreesToEncoderTicks(20)) {

            }
            rotateServo.setPosition(.135);
            sleep(1000);
            dropperServo.setPosition(.0);
            sleep(1000);
            rotateServo.setPosition(0);
            sleep(1000);
            dropperServo.setPosition(.3);
            sleep(1000);
            rotateServo.setPosition(0);
            sleep(1000);
            dropperServo.setPosition(.3);
            slideMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
            while (Math.abs(slideMotor.getCurrentPosition() - convertDegreesToEncoderTicks(0)) > convertDegreesToEncoderTicks(20)) {

            }

        }

    }
    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees / 360 * ENCODER_TICKS_PER_ROTATION);
    }

}

