package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wheels;

@TeleOp(name = "Robot Oriented Wheels Test", group = "Test")
//Uncomment the line below to disable this op
//@Disabled
public class RobotOrientedWheelsTest extends LinearOpMode {
    // Declare variables you will be using throughout this class here

    Wheels wheels; // Declare the wheels class to control the wheels

    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here

        wheels = new Wheels(this, null);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP). Logic once game starts here
        while (opModeIsActive()) {

            wheels.driveByJoystickRobotOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}