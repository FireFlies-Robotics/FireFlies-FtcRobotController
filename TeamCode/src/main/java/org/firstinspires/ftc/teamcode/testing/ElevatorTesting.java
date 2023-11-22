package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.transfer.Elevator;


@TeleOp(name="ElevatorTesting", group="Linear OpMode")
@Disabled
public class ElevatorTesting extends LinearOpMode {
    Elevator elevator;
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        elevator.initElevator();

        while (opModeIsActive()) {
            elevator.moveElevator(gamepad2.right_trigger);
            elevator.moveElevator(-gamepad2.left_trigger);

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0){
                elevator.stopElevator();
            }
        }
    }
}


