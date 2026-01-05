package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class autonBlueGoal extends LinearOpMode {

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

    double curr_time;

    double last_time;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        cameraLibrary.initializeAprilTag();

        //start of the auton
        waitForStart();

        movement.moveBackward(48, 0.5);

        sleep(200);

        cameraLibrary.detectIfShotPossible();

        movement.launchLittleBoy();

        sleep(500);

        movement.turnLeft(45, 0.5);

        sleep(200);

        movement.moveRight(24,0.5);





    }
}
