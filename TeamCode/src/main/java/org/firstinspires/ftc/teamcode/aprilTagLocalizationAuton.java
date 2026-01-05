package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;

@Autonomous
public class aprilTagLocalizationAuton extends LinearOpMode {

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public cameraLibrary cameraLibrary;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;

    public driverCentricMovement driverCentricMovement;

    public movement movement;

    public detectedId motif = detectedId.UNKNOWN;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);
        sensorLibrary = new sensorLibrary(mainLibrary);
        driverCentricMovement = new driverCentricMovement(mainLibrary);
        cameraLibrary = new cameraLibrary(this, mainLibrary);
        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);
        cameraLibrary.initializeAprilTag();

        telemetry.addData("Yaw angle", cameraLibrary.yawAccordingToAprilTag());

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (cameraLibrary.detectID() == detectedId.BLUE_GOAL) {

                    movement.turnLeft(360, .2);

                    //mainLibrary.waitForMotors();

                    //cameraLibrary.detectIfShotPossible();

                }

                telemetry.update();

            }
        }



    }
}
