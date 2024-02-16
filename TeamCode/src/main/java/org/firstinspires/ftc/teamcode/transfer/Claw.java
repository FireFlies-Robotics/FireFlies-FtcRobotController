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

    final private double CLAW_CLOSED_POSITION = 0;
    final private double CLAW_OPEN_POSITION = .3;

    public void initClaw(){
        rightClawServo =  opMode.hardwareMap.get(Servo.class, "rightClawServo");
        leftClawServo = opMode.hardwareMap.get(Servo.class, "leftClawServo");

        leftClawServo.setDirection(Servo.Direction.REVERSE);
        closeClawLeft();
        closeClawRight();

    }
    public void closeClawLeft() {
        leftClawServo.setPosition(CLAW_CLOSED_POSITION);
    }
    public void closeClawRight() {
        rightClawServo.setPosition(CLAW_CLOSED_POSITION);
    }

    public void openClawLeft(){
        leftClawServo.setPosition(CLAW_OPEN_POSITION);
    }
    public void openClawRight(){
        rightClawServo.setPosition(CLAW_OPEN_POSITION);
    }

    public boolean isOpenLeft() {
        return leftClawServo.getPosition() > 0;
    }

    public boolean isOpenRight() {
        return rightClawServo.getPosition() > 0;
    }

}
