package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wheels;

//import org.firstinspires.ftc.teamcode.hardware.Wheels;

@TeleOp(name = "Robot Oriented Wheel Testing", group = "Testing")
//Uncomment the line below to disable this op
@Disabled
public class WheelRobotOrientedTesting extends LinearOpMode {
    // Declare variables you will be using throughout this class here

    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();

    private double xAxis = 0;
    private double yAxis = 0;
    private double rot = 0;
    private Wheels wheels;

    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here
        telemetry.addData("Status", "Initializing");

        wheels = new Wheels(this, null);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP). Logic once game starts here
        while (opModeIsActive()) {
            xAxis = gamepad1.left_stick_x;
            yAxis = -gamepad1.left_stick_y;
            rot = gamepad1.right_stick_x;

            wheels.driveByJoystickFieldOriented(xAxis, yAxis, rot);

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

