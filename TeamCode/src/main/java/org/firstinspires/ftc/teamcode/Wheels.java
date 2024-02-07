package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Wheels {

//    public static final double TICKS_PER_ROTATION = 3611.2; // נמדד על ידי סיבוב הרובוט 20 פעם
//
//    public static final double WHEEL_DIAMETER_CM = 9.6;
//    public static final double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * Math.PI;
//    public static final double MOTOR_ENCODER_RESOLUTION = 537.7;
//    public static final double TICKS_PER_CM = MOTOR_ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE_CM;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    final double kPAngle = .02;
    final double kPDrive = .06;
    final double RANGE = 5;

    private final LinearOpMode opMode; // The opmode used to get the wheels
    private final IMU imu; // Gyros used to get the robots rotation

    double maxSpeed = 1;

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public Wheels(LinearOpMode opMode, IMU imu) {
        this.opMode = opMode;
        this.imu = imu;

        // Getting the wheel motors and setting them up

        frontLeft = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "leftRear");
        backRight = opMode.hardwareMap.get(DcMotor.class, "rightRear");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveByJoystickFieldOriented(double x, double y, double rot) {
        double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Get the yaw angle of the robot
        double X = x;
        double Y = y;
        double ROT = rot;
        if (Math.abs(X) <= 0.1){
            X = 0;
        }
        if (Math.abs(Y) <= 0.1){
            Y = 0;
        }
        if (Math.abs(ROT) <= 0.1){
            ROT  = 0;
        }
        // Rotate the movement direction counter to the robot's rotation
        double rotX = x * Math.cos(yaw) - y * Math.sin(yaw);
        double rotY = x * Math.sin(yaw) + y * Math.cos(yaw);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + ROT) / denominator;
        double backLeftPower = (rotY - rotX + ROT) / denominator;
        double frontRightPower = (rotY - rotX - ROT) / denominator;
        double backRightPower = (rotY + rotX - ROT) / denominator;

        // Applying forces to wheel motors

        frontLeft.setPower(frontLeftPower*maxSpeed);
        backLeft.setPower(backLeftPower*maxSpeed);
        frontRight.setPower(frontRightPower*maxSpeed);
        backRight.setPower(backRightPower*maxSpeed);
    }

    public void driveRobotOriented(double x, double y, double rot) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
        double frontLeftPower = (y + x + rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public boolean autoAdjust(ArrayList<AprilTagDetection> tags) {
        int minId = -1;
        double minRange = 0;
        double yaw = 0;

        for (AprilTagDetection tag : tags) {
            double range = tag.ftcPose.range * 2.54;

            if (minId == -1) {
                minId = tag.id;
                minRange = range;
                yaw = tag.ftcPose.yaw;
            } else if (minRange > range) {
                minId = tag.id;
                minRange = range;
                yaw = tag.ftcPose.yaw;
            }
        }

        if (Math.abs(yaw) < 2) {
            yaw = 0;
        }

        driveRobotOriented(0, (minRange - RANGE) * kPDrive, yaw * kPAngle);

        opMode.telemetry.addData("Yaw", yaw);
        opMode.telemetry.addData("Range", minRange);

        return false;
    }
}