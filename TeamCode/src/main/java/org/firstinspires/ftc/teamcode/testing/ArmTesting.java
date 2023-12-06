package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.transfer.Arm;

@TeleOp(name="ArmTesting", group="Linear OpMode")
@Disabled
public class ArmTesting extends LinearOpMode {

    Arm arm;
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        arm.initArm();

        while (opModeIsActive()) {

            if (gamepad2.x) {
                arm.closeArm();
            }
            if (gamepad2.triangle){
                arm.openArm();
            }
        }
    }}