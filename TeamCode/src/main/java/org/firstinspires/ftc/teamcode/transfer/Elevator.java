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

        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    public void setElevatorDown(){
        leftElevatorMotor.setTargetPosition(0);
        rightElevatorMotor.setTargetPosition(0);

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Tells the motor to run to the specific position
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Tells the motor to run to the specific position


    }

}
