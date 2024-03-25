package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Wheels {

    public static final double TICKS_PER_ROTATION = 3611.2; // נמדד על ידי סיבוב הרובוט 20 פעם

    public static final double WHEEL_DIAMETER_CM = 9.6;
    public static final double WHEEL_CIRCUMFERENCE_CM = WHEEL_DIAMETER_CM * Math.PI;
    public static final double MOTOR_ENCODER_RESOLUTION = 537.7;
    public static final double TICKS_PER_CM = MOTOR_ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE_CM;
    final PID pidAngle = new PID(.03, 0.005, 0.01, 0);
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final LinearOpMode opMode; // The opmode used to get the wheels
    private final IMU imu; // Gyros used to get the robots rotation
    public int weelsCurrentPosition = 0;
    ElapsedTime elapsedTime = new ElapsedTime();
    PID pidDriveY = new PID(.007, 0.01, 0.01, 0);
    PID pidDriveX = new PID(.007, 0.005, 0.01, 0);
    double maxSpeed = 1;

    public Wheels(LinearOpMode opMode, IMU imu) {
        this.opMode = opMode;
        this.imu = imu;
        pidDriveY.setTolerance(2);
        pidDriveX.setTolerance(2);
        pidAngle.setTolerance(2);
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

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public void driveByJoystickFieldOriented(double x, double y, double rot) {
        double yaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Get the yaw angle of the robot
        double X = x;
        double Y = y;
        double ROT = rot;
        if (Math.abs(X) <= 0.1) {
            X = 0;
        }
        if (Math.abs(Y) <= 0.1) {
            Y = 0;
        }
        if (Math.abs(ROT) <= 0.1) {
            ROT = 0;
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

        frontLeft.setPower(frontLeftPower * maxSpeed);
        backLeft.setPower(backLeftPower * maxSpeed);
        frontRight.setPower(frontRightPower * maxSpeed);
        backRight.setPower(backRightPower * maxSpeed);
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

        opMode.telemetry.addData("FrontLeftPower", frontLeftPower);
        opMode.telemetry.addData("BackLeftPower", backLeftPower);
        opMode.telemetry.addData("FrontRightPower", frontRightPower);
        opMode.telemetry.addData("BackRightPower", backRightPower);
    }
//
//    public boolean autoAdjust(ArrayList<AprilTagDetection> tags, double targetYaw, double targetRange) {
//        return autoAdjust(tags, -1, targetYaw, targetRange);
//    }

    public boolean autoAdjust(ArrayList<AprilTagDetection> tags, int targetId, double targetYaw, double targetY, double targetX) {
        int minId = -1;
        double minY = 0;
        double yaw = 0;
        double minX = 0;
        int aprilTagCount = 0;
        ArrayList<Integer> tagsFound = new ArrayList<>();

        for (AprilTagDetection tag : tags) {
            double y = tag.ftcPose.y * 2.54;
            double x = tag.ftcPose.x * 2.54;
            aprilTagCount++;
            tagsFound.add(tag.id);

            if (tag.id == targetId) {
                minId = tag.id;
                minY = y;
                yaw = tag.ftcPose.yaw;
                minX = x;
                break;
            } else if (minId == -1) {
                minId = tag.id;
                minY = y;
                yaw = tag.ftcPose.yaw;
                minX = x;
            } else if (minY > y) {
                minId = tag.id;
                minY = y;
                yaw = tag.ftcPose.yaw;
                minX = x;
            }
        }

        StringBuilder tagsFoundStr = new StringBuilder();
        for (int id : tagsFound) {
            tagsFoundStr.append(id).append(", ");
        }

        opMode.telemetry.addData("Tags Found", tagsFoundStr.toString());
        opMode.telemetry.addData("Locating Based on", minId);

        if (minId == -1) return false;
        double driveSpeedY = -pidDriveY.calculate(minY, targetY);
        double driveSpeedX = pidDriveX.calculate(minX, 0);
        double rotationSpeed = pidAngle.calculate(yaw, targetYaw);
        driveRobotOriented(driveSpeedX, driveSpeedY, rotationSpeed);
        boolean returnCondition = (pidDriveY.atSetPoint() && pidDriveX.atSetPoint() && pidAngle.atSetPoint());

        opMode.telemetry.addData("Yaw", yaw);
        opMode.telemetry.addData("Y", minY);
        opMode.telemetry.addData("X", minX);
        opMode.telemetry.addData("Y Difference", targetY - minY);
        opMode.telemetry.addData("X Difference", -minX);
        opMode.telemetry.addData("Yaw Difference", Math.abs(yaw - targetYaw));
        opMode.telemetry.addData("April Tags", aprilTagCount);
        opMode.telemetry.addData("Drive Speed", driveSpeedY);
        opMode.telemetry.addData("Drive Rotation", rotationSpeed);

        return returnCondition;
    }

    public void runWithEncoder() {
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateByEncoder(double degrees, double power){
        rotateByEncoder(degrees,power,5);
    }
    public void rotateByEncoder(double degrees, double power, double seconds) {
        elapsedTime.reset();

        double toPosition = degrees / 360 * TICKS_PER_ROTATION;
        driveWheelToPosition(frontRight, power, -toPosition);
        driveWheelToPosition(frontLeft, power, toPosition);
        driveWheelToPosition(backRight, power, -toPosition);
        driveWheelToPosition(backLeft, power, toPosition);
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && motorsIsBussy() && elapsedTime.seconds() <= seconds) {
            opMode.telemetry.addData("Front Position", "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position", "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            //opMode.idle();
        }
        stop();
    }
    public void driveForward(double distanceCM, double power){
        driveForward(distanceCM,power,5);
    }
    public void driveForward(double distanceCM, double power,double seconds) {
        elapsedTime.reset();
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight, power, toPosition);
        driveWheelToPosition(frontLeft, power, toPosition);
        driveWheelToPosition(backRight, power, toPosition);
        driveWheelToPosition(backLeft, power, toPosition);
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && motorsIsBussy() && elapsedTime.seconds() <= seconds) {
            opMode.telemetry.addData("Front Position", "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position", "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            if (distanceCM <= distanceCM / 5) {
                power = power / 5;
            }
//
//            if (power == 0){stop();}
//            opMode.idle();
        }
        stop();
    }
    public void driveBackword(double distanceCM, double power)
    {
        driveBackword(distanceCM,power,5);
    }
    public void driveBackword(double distanceCM, double power,double seconds) {
        elapsedTime.reset();
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight, power, -toPosition);
        driveWheelToPosition(frontLeft, power, -toPosition);
        driveWheelToPosition(backRight, power, -toPosition);
        driveWheelToPosition(backLeft, power, -toPosition);
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && motorsIsBussy() && elapsedTime.seconds()<=seconds) {
            opMode.telemetry.addData("Front Position", "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position", "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());
            opMode.telemetry.update();
            //opMode.idle();
        }
        stop();
    }


    public void driveLeft(double distanceCM, double power) {
        driveLeft(distanceCM,power,5);
    }
    public void driveLeft(double distanceCM, double power,double seconds) {
        elapsedTime.reset();
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight, power, toPosition);
        driveWheelToPosition(frontLeft, power, -toPosition);
        driveWheelToPosition(backRight, power, -toPosition);
        driveWheelToPosition(backLeft, power, toPosition);
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && motorsIsBussy()) {
            opMode.telemetry.addData("Front Position", "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position", "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());

            opMode.telemetry.update();
            //opMode.idle();
        }
        stop();
    }

    public void driveRight(double distanceCM, double power) {
        driveRight(distanceCM,power,5);

    }
    public void driveRight(double distanceCM, double power ,double seconds) {
        elapsedTime.reset();
        double toPosition = distanceCM * TICKS_PER_CM;
        driveWheelToPosition(frontRight, power, -toPosition);
        driveWheelToPosition(frontLeft, power, toPosition);
        driveWheelToPosition(backRight, power, toPosition);
        driveWheelToPosition(backLeft, power, -toPosition);
        while (opMode.opModeIsActive() && !opMode.isStopRequested() && motorsIsBussy() && elapsedTime.seconds() <= seconds) {
            opMode.telemetry.addData("Front Position", "%7d :%7d",
                    frontRight.getCurrentPosition(),
                    frontLeft.getCurrentPosition());
            opMode.telemetry.addData("Back Position", "%7d :%7d",
                    backRight.getCurrentPosition(),
                    backLeft.getCurrentPosition());

            opMode.telemetry.update();


//            opMode.idle();
        }
        stop();
    }

    public boolean motorsIsBussy() {
        return frontLeft.isBusy() ||
                frontRight.isBusy() ||
                backLeft.isBusy() ||
                backRight.isBusy();
    }

    public void driveByEncoder() {
    }

    private void driveWheelToPosition(DcMotor wheel, double power, double toPosition) {
        wheel.setTargetPosition((int) Math.round(toPosition) + wheel.getCurrentPosition());
        wheel.setPower(power);
        wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}