package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedInputStream;
@Disabled
@Autonomous(name="GoToPosition", group="Robot")

public class GoToPosition {

    /*public static void goToPosition(double x, double y, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, Odometry.java odometry) {
        goToX(x, frontLeft, backLeft, frontRight, backRight, odometry);
        goToY(y, frontLeft, backLeft, frontRight, backRight, odometry);
    }*/
    public static void goToPosition(double x, double y, Odometry odometry, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        double currentX = odometry.getXCoordinate();
        double currentY = odometry.getYCoordinate();
        double currentHeading = odometry.getHeading();
        double changeX = x - currentX;
        double changeY = y - currentY;
        double angle = Math.atan2(changeY, changeX);
        rotate(angle, odometry, leftFront, leftBack, rightFront, rightBack);
        goToY(y, leftFront, leftBack, rightFront, rightBack, odometry);
    }

    public static void rotate(double goalAngle, Odometry odometry, DcMotor leftFront, DcMotor leftBack, DcMotor rightFront, DcMotor rightBack) {
        double currentHeading = odometry.getHeading();
        while (goalAngle != currentHeading) {
            currentHeading = odometry.getHeading(); // Get heading and check how close we are
                leftFront.setPower(1);
                leftBack.setPower(1);
                rightFront.setPower(-1);
                rightBack.setPower(-1);  // Set powers to rotate
        }
    }

    public static double[] goToX(double x, Odometry odometry) {
        /**
         * returns a list of doubles in the following order:
         * frontLEFT
         * BACKLEFT
         * FRONTRIGHT
         * BACKRIGHT
         * Distance
         */




        double p = 15; //Stopping distance
        double currentX = odometry.getXCoordinate(); // where we are now
        double distance = x - currentX; // how far the robot is form the input
        double power = distance / p;

            if (power > 1)
                power = 1;
            else if (power < -1)
                power = -1;

            power = power/2.0;

            return new double[]{power, -power, -power, power, distance};

    }

    public static void goToY(double y, DcMotor frontLeft, DcMotor backLeft, DcMotor frontRight, DcMotor backRight, Odometry odometry) {
        double p = 15; //Stopping distance
        double currentY = odometry.getYCoordinate(); // where we are now
        double distance = y - currentY; // how far the robot is form the input
        double power = distance / p;

        while (distance > 0.25) {

            currentY = odometry.getYCoordinate(); // where we are now
            distance = y - currentY; // how far the robot is form the input
            power = distance / p;

            if (power > 1)
                power = 1;
            else if (power < -1)
                power = -1;

            power = power/2.0;

            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
        }
    }

}
