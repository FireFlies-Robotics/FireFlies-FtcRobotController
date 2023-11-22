package org.firstinspires.ftc.teamcode.UnloadSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class UnloadSystem {
    private final LinearOpMode opMode;

    DcMotor leftElevatorMotor;
    DcMotor rightElevatorMotor;

    public UnloadSystem(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void initUnlodSystem() {
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

