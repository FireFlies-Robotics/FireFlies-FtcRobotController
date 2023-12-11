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
    }

    void rotate(double rotationValue) {
        frontRight.setPower(-rotationValue);
        backRight.setPower(-rotationValue);
        frontLeft.setPower(rotationValue);
        backLeft.setPower(rotationValue);
    }
}
