package org.firstinspires.ftc.teamcode.eocv_testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Wheels;

public class AutoWheels extends Wheels {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    OpMode opMode;
    IMU imu;

    public AutoWheels(LinearOpMode opMode, IMU imu) {
        super((LinearOpMode) opMode, imu);

        this.opMode = opMode;
        this.imu = imu;


        frontLeft = opMode.hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "BackRight");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void rotate(double rotationValue) {
        frontRight.setPower(-rotationValue);
        backRight.setPower(-rotationValue);
        frontLeft.setPower(rotationValue);
        backLeft.setPower(rotationValue);
    }

    void forwards() {
        frontRight.setPower(.6);
        backRight.setPower(.6);
        frontLeft.setPower(.6);
        backLeft.setPower(.6);
    }

    void operate(double forwardsValue, double rotationValue) {
        double fr = forwardsValue - rotationValue;
        double br = forwardsValue - rotationValue;
        double fl = forwardsValue + rotationValue;
        double bl = forwardsValue + rotationValue;

        double norm = Math.max(Math.max(Math.abs(fr), Math.abs(br)), Math.max(Math.abs(fr), Math.abs(br)));

        if (norm > 1) {
            fr /= norm;
            br /= norm;
            fl /= norm;
            bl /= norm;
        }

        frontRight.setPower(fr);
        backRight.setPower(br);
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
    }
}
