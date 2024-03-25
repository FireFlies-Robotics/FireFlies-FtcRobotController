
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.transfer.Arm;
import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Elevator;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="Main Linear OpMode", group="Linear OpMode")
//Uncomment the line below to disable this op
//@Disabled
public class MainOpMode extends LinearOpMode {
    // Declare variables you will be using throughout this class here

    // Time that runs since the program began running
    private ElapsedTime runtime = new ElapsedTime();
    Arm arm;

    boolean lastStateLeft = false;
    boolean lastStateRight = false;

    Claw claw;
    Elevator elevator;
    Intake intake;

    AirPlane airPlane;
    Wheels wheels; // Declare the wheels class to control the wheels

    IMU imu; // Declare class for getting robot angles
    VisionPortal portal;
    AprilTagProcessor processor;

    @Override
    public void runOpMode() {
        // Runs when init is pressed. Initialize variables and pregame logic here
        arm = new Arm(this);
        claw = new Claw(this);
        intake = new Intake(this);
        elevator = new Elevator(this);
        airPlane = new AirPlane(this);
        arm.initArm();
        claw.initClaw();
        elevator.initElevator();
        intake.initIntake();
        airPlane.initAirPkane();
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(webcamName, processor);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
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

            if (gamepad2.cross && !isStopRequested() && opModeIsActive()) {
                arm.closeArm();
            }

            if (gamepad2.triangle&& !isStopRequested() && opModeIsActive()) {
                arm.openArm();
            }

            if (gamepad2.options&& !isStopRequested() && opModeIsActive()) {
                arm.midArm();
            }
            if (gamepad2.share&& !isStopRequested() && opModeIsActive()){
                elevator.climb();
            }

            if (gamepad2.left_bumper && !lastStateLeft && claw.isOpenRight()&& !isStopRequested() && opModeIsActive()) {
                claw.closeClawRight();
                lastStateLeft = true;
            } else if (gamepad2.left_bumper && !lastStateLeft && !claw.isOpenRight()&& !isStopRequested() && opModeIsActive()) {
                claw.openClawRight();
                lastStateLeft = true;
            } else if (!gamepad2.left_bumper&& !isStopRequested() && opModeIsActive()) {
                lastStateLeft = false;
            }

            if (gamepad2.right_bumper && !lastStateRight && claw.isOpenLeft()&& !isStopRequested() && opModeIsActive()) {
                claw.closeClawLeft();
                lastStateRight = true;
            } else if (gamepad2.right_bumper && !lastStateRight && !claw.isOpenLeft()&& !isStopRequested() && opModeIsActive()) {
                claw.openClawLeft();
                lastStateRight = true;
            } else if (!gamepad2.right_bumper&& !isStopRequested() && opModeIsActive()) {
                lastStateRight = false;
            }

            elevator.moveElevator(gamepad2.right_trigger - gamepad2.left_trigger);

//            if (gamepad2.right_trigger <= 0.1 && gamepad2.left_trigger <= 0.1){
//                elevator.stopElevator();
//            }

            // Reset the robot's default angle with options button
            if (gamepad1.options&& !isStopRequested() && opModeIsActive()) {
                imu.resetYaw();
            }

            // Slow the robot down when the left bumber is pressed
            if (gamepad1.right_bumper && !isStopRequested() && opModeIsActive()) {
                wheels.setMaxSpeed(.3);
            } else {
                wheels.setMaxSpeed(1);
            }
            if (gamepad1.left_bumper&& !isStopRequested() && opModeIsActive()) {
                wheels.setMaxSpeed(.5);
            } else {
                wheels.setMaxSpeed(1);
            }
            if (gamepad2.circle&& !isStopRequested() && opModeIsActive()){
                intake.intake();
                claw.openClawRight();
                claw.openClawLeft();
            } else if (gamepad2.square&& !isStopRequested() && opModeIsActive()){
                intake.outake();
            } else {
                intake.stop();
            }
            // Move robot by controller 1
            if ( !isStopRequested() && opModeIsActive()) {
                if (gamepad1.share){
                  //  wheels.autoAdjust(processor.getDetections(), 0, 20);
                }
                else {
                    wheels.driveByJoystickFieldOriented(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
                }
            }
            if (gamepad1.touchpad){
                airPlane.lunchAirPlane();
            }
            // Show data on driver station
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed", wheels.getMaxSpeed()*100 + "%");
            telemetry.update();
        }
    }
}
