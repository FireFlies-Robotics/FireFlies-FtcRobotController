package org.firstinspires.ftc.teamcode.transfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class Elevator {
    private final LinearOpMode opMode;

    DcMotor leftElevatorMotor;
    DcMotor rightElevatorMotor;

    public Elevator(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initElevator() {
        rightElevatorMotor = opMode.hardwareMap.get(DcMotor.class, "rightElevatorMotor");
        leftElevatorMotor = opMode.hardwareMap.get(DcMotor.class, "leftElevatorMotor");
        leftElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.telemetry.addData("Hardware: ", "initialized");
        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftElevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void moveElevator(double motorPower) { // this function make the elevator speed
        if (Math.abs(motorPower ) > .1) {
            leftElevatorMotor.setPower(motorPower);
            rightElevatorMotor.setPower(motorPower);
        } else {
            leftElevatorMotor.setPower(0);
            rightElevatorMotor.setPower(0);
        }

        opMode.telemetry.addData("Motor Power", String.valueOf(leftElevatorMotor.getPower()));
    }
    public void climb(){

        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftElevatorMotor.setTargetPosition(0);
        rightElevatorMotor.setTargetPosition(0);
        while (opMode.opModeIsActive()){
            rightElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftElevatorMotor.setPower(-1);
            rightElevatorMotor.setPower(-1);

        }
    }
    public void elevatorUp(int ticks){


        leftElevatorMotor.setTargetPosition(ticks);
        rightElevatorMotor.setTargetPosition(ticks);

        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElevatorMotor.setPower(1);
        rightElevatorMotor.setPower(1);
        opMode.telemetry.addData("ticks", ticks);
        opMode.telemetry.update();
    }
    public void stopElevator(){


    }
    public void setElevatorDown(){

        leftElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftElevatorMotor.setTargetPosition(0);
        rightElevatorMotor.setTargetPosition(0);



    }

}
