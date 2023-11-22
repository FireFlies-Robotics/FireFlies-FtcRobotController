package org.firstinspires.ftc.teamcode.transfer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo rightClawServo;
    private Servo leftClawServo;

    final private float CLAW_CLOSED_POSITION = 0;
    final private float CLAW_OPEN_POSITION = 1;

    public void initClaw(){
        rightClawServo.setDirection(Servo.Direction.REVERSE); //todo check which servo needs to be reversed
        closeClaw();

    }
    public void closeClaw(){ //todo check which position is close and open
        rightClawServo.setPosition(CLAW_CLOSED_POSITION);
        leftClawServo.setPosition(CLAW_CLOSED_POSITION);
    }


}
