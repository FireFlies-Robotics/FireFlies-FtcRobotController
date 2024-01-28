package org.firstinspires.ftc.teamcode.transfer;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private LinearOpMode opMode;
    private DcMotor intakeMotor;
    public double intakeMotoeOff = 0;
    private double currenIntakeSpeed = 0;

    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void initIntake(){ // where every part in the system moves when press init
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(intakeMotoeOff);
    }
    public void activateIntake(){
        if (intakeMotor.getPower() == 1){
            intakeMotor.setPower(0);
        }
        else{
            intakeMotor.setPower(1 );
        }
    }
    public void intakeSpeedUp(){ // speed up the motor in 0.1
        intakeMotor.setPower(currenIntakeSpeed + 0.1);
    }
    public void intakeSpeedDown(){ // speed down the motor in 0.1
        intakeMotor.setPower(currenIntakeSpeed + 0.1);

    }
}
