package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedInputStream;

@Autonomous(name="Odometry Test", group="Robot")

public class GlobalCoordinateTracking extends LinearOpMode {
    private DcMotor verticalLeft, verticalRight, horizontal;
    String verticalLeftEncoderName = "backLeft";
    String verticalRightEncoderName = "backRight";
    String horizontalEncoderName = "frontLeft";

    public void runOpMode() {

         verticalLeft = hardwareMap.dcMotor.get( verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Odometry odometry = new Odometry(verticalLeft, verticalRight, horizontal);

        waitForStart();

        while (opModeIsActive()) {  

            odometry.updatePosition();

            telemetry.addData("XCoord", odometry.getXCoordinate());
            telemetry.addData("YCoord", odometry.getYCoordinate());
            telemetry.addData("Heading", odometry.getHeading());
            telemetry.addData("Leftencoder", verticalLeft.getCurrentPosition());
            telemetry.addData("Rightencoder", verticalRight.getCurrentPosition());
            telemetry.addData("Middle1encoder", horizontal.getCurrentPosition());
            telemetry.update();

        }
    }

}

