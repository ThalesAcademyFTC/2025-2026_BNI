package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class autonBlueFar extends LinearOpMode {

    public driverCentricMovement driverCentricMovement;

    public fieldCentricMovement fieldCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public cameraLibrary cameraLibrary;

    public movement movement;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;

    public detectedId motif = detectedId.UNKNOWN;

    public String motifPattern;

    double speed = 0.5;

    double X;

    double Y = 112;

    double YAW = 20;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        cameraLibrary.initializeAprilTag();

        //start of the auton
        waitForStart();

        movement.moveForward(7, speed);

        movement.turnLeft(25, speed);

        sleep(200);

        movement.moveForward(12, speed);

        sleep(200);

        movement.turnLeft(10, .1);

        sleep(200);

        cameraLibrary.autoPositionBlue(X,Y,YAW);

        mainLibrary.restMotors();

        sleep(200);

        movement.moveLeft(2, speed);

        sleep(200);

        movement.launchLittleBoy(1);

        sleep(500);

        movement.turnRight(24, speed);

        sleep(200);

        movement.moveLeft(24, speed);


    }
}
