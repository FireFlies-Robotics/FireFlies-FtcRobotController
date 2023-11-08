package org.firstinspires.ftc.teamcode.UnloadSystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "UnloadSystem")
public class UnloadSystem extends OpMode {

        DcMotor motor;
        @Override
        public void init() {
            motor = hardwareMap.get(DcMotor.class, "motor");
            telemetry.addData("Hardware: ", "initialized");



    }

    @Override
    public void loop() {

        float x = gamepad1.right_trigger;
        if (gamepad1.right_trigger > 0){
                motor.setPower(x);
            }
            motor.setPower(0);
    }
}