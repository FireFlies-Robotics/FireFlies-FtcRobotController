package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.transfer.Arm;
import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Elevator;
import org.firstinspires.ftc.teamcode.transfer.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Aytonomus {





    @Autonomous(name="Autonomous", group="Linear OpMode")
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

        VisionPortal portal;
        AprilTagProcessor processor;
        @Override
        public void runOpMode() {
            // Runs when init is pressed. Initialize variables and pregame logic here
            arm = new Arm(this);
            claw = new Claw(this);
            intake = new Intake(this);
            elevator = new Elevator(this);
            arm.initArm();
            claw.initClaw();
            elevator.initElevator();
            intake.initIntake();


            // Retrieve the IMU from the hardware map
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            wheels = new Wheels(this, imu);



            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            while (opModeIsActive ()) {
//                if(){}
//                if(){}
//                if(){}
                wheels.driveForward(60, 0.6);
                claw.openClawLeft();
                wheels.rotateByEncoder(90, 0.4);
                arm.openArm();
                wheels.autoAdjust(processor.getDetections());

                elevator.elevatorUp(700);
                claw.openClawRight();


            }
        }
    }

}
