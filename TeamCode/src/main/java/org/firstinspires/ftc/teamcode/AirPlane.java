package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
public class AirPlane {

    private final LinearOpMode opMode;

    private Servo airPlaneServo;

    public AirPlane (LinearOpMode opMode) {this.opMode = opMode;}

    public void initAirPkane(){
        airPlaneServo =  opMode.hardwareMap.get(Servo.class, "airPlane");
        airPlaneServo.setPosition(0);

    }


    public void lunchAirPlane(){
        airPlaneServo.setPosition(1);
    }

}
