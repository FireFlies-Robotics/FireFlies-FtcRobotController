package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class BluePurpleYellow extends LinearOpMode {


    MainAutonomus mainAutonomus;

    @Override
    public void runOpMode() throws InterruptedException {
        mainAutonomus = new MainAutonomus(false,this );
        mainAutonomus.runPeuple();
        mainAutonomus.blueYellowPixel();
    }
}
