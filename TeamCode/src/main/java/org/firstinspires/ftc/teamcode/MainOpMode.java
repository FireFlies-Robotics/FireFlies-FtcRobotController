
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.transfer.Arm;
import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Elevator;
import org.firstinspires.ftc.teamcode.transfer.Intake;


@TeleOp(name="Main Linear OpMode", group="Linear OpMode")
//Uncomment the line below to disable this op
//@Disabled
public class MainOpMode extends LinearOpMode {
    // Declare variables you will be using throughout this class here

    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();
    Arm arm;
    Claw claw;
    Elevator elevator;
    Intake intake;

    Wheels wheels; // Declare the wheels class to control the wheels

    IMU imu; // Declare class for getting robot angles
    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here
        arm.initArm();
        claw.initArm();
        elevator.initElevator();
        intake.initIntake();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        wheels = new Wheels(this, imu);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Speed", "Waiting to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP). Logic once game starts here
        while (opModeIsActive()) {

            if( gamepad2.touchpad){
                claw.openClaw();
                arm.closeArm();
                elevator.setElevatorDown();
            }
            if (gamepad2.x) {
                arm.closeArm();
            }
            if (gamepad2.triangle){
                arm.openArm();
            }

            if (gamepad2.circle) {
                claw.closeClaw();
            }
            if (gamepad2.square){
                claw.openClaw();
            }

            elevator.moveElevator(gamepad2.right_trigger);
            elevator.moveElevator(-gamepad2.left_trigger);

            if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0){
                elevator.stopElevator();
            }

            // Reset the robot's default angle with options button
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Slow the robot down when the left bumber is pressed
            if (gamepad1.left_bumper) {
                wheels.setMaxSpeed(.5);
            } else {
                wheels.setMaxSpeed(1);
            }

            if(gamepad2.right_bumper){
                intake.intakeSpeedUp();
            }
            if(gamepad2.left_bumper){
                intake.intakeSpeedDown();
            }
            // Move robot by controller 1
            wheels.driveByJoystickFieldOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed", wheels.getMaxSpeed()*100 + "%");
            telemetry.update();
        }
    }
}
