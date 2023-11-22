package org.firstinspires.ftc.teamcode.transfer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final LinearOpMode opMode;

    private Servo rightClawServo;
    private Servo leftClawServo;
    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    final private float CLAW_CLOSED_POSITION = 0;
    final private float CLAW_OPEN_POSITION = 1;

    public void initArm(){
        rightClawServo =  opMode.hardwareMap.get(Servo.class, "rightClawServo");
        leftClawServo = opMode.hardwareMap.get(Servo.class, "leftClawServo");

        rightClawServo.setDirection(Servo.Direction.REVERSE); //todo check which servo needs to be reversed
        closeClaw();

    }
    public void closeClaw(){ //todo check which position is close and open
        rightClawServo.setPosition(CLAW_CLOSED_POSITION);
        leftClawServo.setPosition(CLAW_CLOSED_POSITION);
    }

    public void openClaw(){
        rightClawServo.setPosition(CLAW_OPEN_POSITION);
        leftClawServo.setPosition(CLAW_OPEN_POSITION);
    }


}