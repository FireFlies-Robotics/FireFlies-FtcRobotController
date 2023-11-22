package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.max;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    private final LinearOpMode opMode;
    private final IMU imu;

    double maxSpeed = 1;

    public Wheels(LinearOpMode opMode, IMU imu) {
        this.opMode = opMode;
        this.imu = imu;

        frontLeft = opMode.hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "BackRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }
    public double[] calculateMecanum(double x, double y, double rot){
        double fr = y - x - rot;
        double br = y + x - rot;
        double fl = y + x + rot;
        double bl = y - x + rot;
        opMode.telemetry.addData("Front Position",  "%7d :%7d",
                frontRight.getCurrentPosition(),
                frontLeft.getCurrentPosition());
        opMode.telemetry.addData("Back Position",  "%7d :%7d",
                backRight.getCurrentPosition(),
                backLeft.getCurrentPosition());
        return new double[]{fr,br,fl,bl};

    }

    public void driveByJoystickRobotOriented(double x, double y, double rot) {
        double[] wheelPower = calculateMecanum(x ,y ,rot);
        double fr = wheelPower[0];
        double br = wheelPower[1];
        double fl = wheelPower[2];
        double bl = wheelPower[3];

        double norm = max(max(abs(fr), abs(br)), max(abs(fr), abs(br)));

        if (norm > 1) {
            fr /= norm;
            br /= norm;
            fl /= norm;
            bl /= norm;
        }

        if (opMode.gamepad1.left_stick_button || opMode.gamepad1.left_trigger>.4) {
            fr *= .25;
            br *= .25;
            fl *= .25;
            bl *= .25;
        } else if (opMode.gamepad1.right_trigger>.4) {
            fr *= .8;
            br *= .8;
            fl *= .8;
            bl *= .8;
        } else {
            fr *= .6;
            br *= .6;
            fl *= .6;
            bl *= .6;
        }

        frontRight.setPower(fr);
        backRight.setPower(br);
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
    }

    public void driveByJoystickFieldOriented(double x, double y, double rot) {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-yaw) - y * Math.sin(-yaw);
        double rotY = x * Math.sin(-yaw) + y * Math.cos(-yaw);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;

        frontLeft.setPower(frontLeftPower*maxSpeed);
        backLeft.setPower(backLeftPower*maxSpeed);
        frontRight.setPower(frontRightPower*maxSpeed);
        backRight.setPower(backRightPower*maxSpeed);
    }
}