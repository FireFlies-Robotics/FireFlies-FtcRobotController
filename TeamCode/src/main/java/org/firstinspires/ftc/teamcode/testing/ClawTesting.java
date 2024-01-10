package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.transfer.Claw;
import org.firstinspires.ftc.teamcode.transfer.Intake;

@TeleOp(name="ClawTesting", group="Linear OpMode")
//    @Disabled
    public class ClawTesting extends LinearOpMode {
    Claw claw = new Claw( this);


        // Declare OpMode members.

        @Override
        public void runOpMode() {
            claw.initClaw();
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {

                if (gamepad2.circle) {
                    claw.closeClaw();
                }
                if (gamepad2.square){
                    claw.openClaw();
                }
            }
        }
    }