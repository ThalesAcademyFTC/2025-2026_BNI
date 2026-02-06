package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class autonRedFar extends LinearOpMode {

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

    double X = -2;

    double Y = 110;

    double YAW = -20;

    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        cameraLibrary.initializeAprilTag();

        //start of the auton
        waitForStart();

        movement.moveForward(13, speed);

        sleep(200);

        movement.turnRight(25, speed);

        sleep(200);

        cameraLibrary.autoPositionRed(X,Y,-28);

        sleep(200);

        movement.moveRight(2, speed);

        sleep(200);

        movement.launchLittleBoy(.86);

        sleep(500);

        movement.turnLeft(24, speed);

        sleep(200);

        movement.moveRight(24, speed);

    }
}
