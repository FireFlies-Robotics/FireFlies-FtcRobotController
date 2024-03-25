package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedPurpleYellow extends LinearOpMode {


    MainAutonomus mainAutonomus;

    @Override
    public void runOpMode() throws InterruptedException {
        mainAutonomus = new MainAutonomus(true,this );
        mainAutonomus.runPeuple();
        mainAutonomus.blueYellowPixel();
    }
}