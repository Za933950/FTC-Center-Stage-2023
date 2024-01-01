package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.GoToPosition.*;


@Autonomous(name="GTP Test", group="Robot")

public class GoToPositionTest extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal, leftBack, leftFront,rightFront, rightBack;

    String verticalLeftEncoderName = "frontLeft";
    String verticalRightEncoderName = "frontRight";
    String horizontalEncoderName = "intakeMotor";

    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("frontLeft");
        leftBack = hardwareMap.dcMotor.get("backLeft");
        rightFront = hardwareMap.dcMotor.get("frontRight");
        rightBack = hardwareMap.dcMotor.get("backRight");

        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal);
        Thread positionUpdate = new Thread(odometry);
        positionUpdate.start();

        waitForStart();

        if (opModeIsActive()) {
            GoToPosition.goToPosition(24, 48,90,10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, 0.8);
            GoToPosition.goToPosition(10,24,90, 10, odometry, leftFront, leftBack, rightFront, rightBack, telemetry, 0.8);

        }
    }

}

