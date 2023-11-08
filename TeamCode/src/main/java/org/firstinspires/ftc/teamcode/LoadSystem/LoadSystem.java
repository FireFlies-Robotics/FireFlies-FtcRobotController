package org.firstinspires.ftc.teamcode.LoadSystem;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LoadSystem {



    private DcMotor intakeMotor;
    public double intakeMotoeOff = 0;
    private double currenIntakeSpeed = 0;


    public void initLoadSystem(){ // where every part in the system moves when press init
         intakeMotor.setPower(intakeMotoeOff);
    }
    public void intakeSpeedUp(){ // speed up the motor in 0.1
        intakeMotor.setPower(currenIntakeSpeed + 0.1);
    }
    public void intakeSpeedDown(){ // speed down the motor in 0.1
        intakeMotor.setPower(currenIntakeSpeed + 0.1);

    }
}
