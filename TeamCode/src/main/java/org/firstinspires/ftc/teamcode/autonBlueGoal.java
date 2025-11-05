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

    public movementClasses movementClasses;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;

    public detectedId motif = detectedId.UNKNOWN;

    public String motifPattern;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary);

        movementClasses = new movementClasses(mainLibrary, driverCentricMovement);

        cameraLibrary.initializeAprilTag();


        //start of the auton
        waitForStart();

        movementClasses.moveBackwardInches(50, .1);

        cameraLibrary.detectIfShotPossible();

        if (opModeIsActive()) {

            while (opModeIsActive() && motif == detectedId.UNKNOWN) {

                motif = cameraLibrary.detectID();

                if (motif == detectedId.GREEN_PURPLE_PURPLE) {
                    mainLibrary.rgbIndicator.setPosition(.444); //red
                    telemetry.addLine("Works");
                    blackboard.put(motifPattern, detectedId.GREEN_PURPLE_PURPLE);
                } else if (motif == detectedId.PURPLE_GREEN_PURPLE) {
                    mainLibrary.rgbIndicator.setPosition(.333); //orange
                    telemetry.addLine("Works");
                    blackboard.put(motifPattern, detectedId.PURPLE_GREEN_PURPLE);
                } else if (motif == detectedId.PURPLE_PURPLE_GREEN) {
                    mainLibrary.rgbIndicator.setPosition(.388); //yellow
                    telemetry.addLine("Works");
                    blackboard.put(motifPattern, detectedId.PURPLE_PURPLE_GREEN);
                } else {
                    mainLibrary.rgbIndicator.setPosition(0);
                }
            }


        }

        blackboard.put("MOTIF", motif);





    }
}
