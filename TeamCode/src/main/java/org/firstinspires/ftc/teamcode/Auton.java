package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class Auton extends LinearOpMode {

    public mainLibrary main;
    public ElapsedTime runtime = new ElapsedTime();

    double rest = 100;



    public void runOpMode() {
        waitForStart();

    }

}
