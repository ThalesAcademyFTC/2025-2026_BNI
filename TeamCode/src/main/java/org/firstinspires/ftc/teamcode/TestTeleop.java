package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TestTeleop extends OpMode {

    public driverCentricMovement driverCentricMovement;

    public fieldCentricMovement fieldCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public cameraLibrary cameraLibrary;

    public detectedId detectedId;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;

    public movement movement;

    double rbtSpd = .5;

    double startPosition = .5;

    double ledValue = 0.277;

    double constant = 1;

    AprilTagPoseFtc pose;

    public static final String MOTIF = "MOTIF";

    Pose3D robotPosition = null;

    boolean THEBOOZE = false;

    double turn = .5;

    detectedId motif;

    public void init() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        cameraLibrary.initializeAprilTag();

    }

    public void loop() {

        telemetry.update();

        /*
        experimental blue goal teleop function that adds aim mode to the teleop controller
        allowing for the camera to scan the april tag and constantly adjust and turn to reach
        the ideal shooting position while still allowing fblr movement for manual shooting
        adjustments.
        */

        cameraLibrary.tagReferencePosition();


    }
}
