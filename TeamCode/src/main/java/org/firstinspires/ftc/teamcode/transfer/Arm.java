package org.firstinspires.ftc.teamcode.transfer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final LinearOpMode opMode;

    private Servo rightArmServo;
    private Servo leftArmServo;
    public Arm(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    final private float ARM_CLOSED_POSITION = 0;
    final private float ARM_OPEN_POSITION = 1;

    public void initArm(){
        rightArmServo =  opMode.hardwareMap.get(Servo.class, "rightArmServo");
        leftArmServo = opMode.hardwareMap.get(Servo.class, "leftArmServo");

        rightArmServo.setDirection(Servo.Direction.REVERSE); //todo check which servo needs to be reversed
        closeArm();

    }
    public void closeArm(){ //todo check which position is close and open
        rightArmServo.setPosition(ARM_CLOSED_POSITION);
        leftArmServo.setPosition(ARM_CLOSED_POSITION);
    }

    public void openClaw(){
        rightArmServo.setPosition(ARM_OPEN_POSITION);
        leftArmServo.setPosition(ARM_OPEN_POSITION);
    }




}
