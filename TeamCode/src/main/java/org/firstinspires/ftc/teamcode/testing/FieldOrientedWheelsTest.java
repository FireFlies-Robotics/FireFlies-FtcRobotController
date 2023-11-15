package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Wheels;

@TeleOp(name = "Field Oriented Wheels Test", group = "Test")
//Uncomment the line below to disable this op
//@Disabled
public class FieldOrientedWheelsTest extends LinearOpMode {
    // Declare variables you will be using throughout this class here

    Wheels wheels; // Declare the wheels class to control the wheels

    IMU imu; // Declare class for getting robot angles

    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here

        wheels = new Wheels(this);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP). Logic once game starts here
        while (opModeIsActive()) {

            wheels.driveByJoystickFieldOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}