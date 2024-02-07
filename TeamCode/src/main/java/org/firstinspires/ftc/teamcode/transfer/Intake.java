package org.firstinspires.ftc.teamcode.transfer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {

    private LinearOpMode opMode;
    private DcMotor intakeMotor;
    public double intakeMotorOff = 0;

    public Intake(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void initIntake() { // where every part in the system moves when press init
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setPower(intakeMotorOff);
    }
    public void intake() {
        intakeMotor.setPower(1);
    }
    public void outake() {
        intakeMotor.setPower(-1);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
}
