package org.firstinspires.ftc.teamcode.transfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class Elevator {
    private final LinearOpMode opMode;

    DcMotor leftElevatorMotor;
    DcMotor rightElevatorMotor;

    public Elevator(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initElevator() {
        leftElevatorMotor = opMode.hardwareMap.get(DcMotor.class, "leftElevatorMotor");
        rightElevatorMotor = opMode.hardwareMap.get(DcMotor.class, "rightElevatorMotor");

        opMode.telemetry.addData("Hardware: ", "initialized");


    }

    public void moveElevator(float motorPower) {

        leftElevatorMotor.setPower(motorPower);
        rightElevatorMotor.setPower(motorPower);
    }
    public void stopElevator(){
        leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
