package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;

@Autonomous
public class testAuton extends LinearOpMode {

    public driverCentricMovement driverCentricMovement;

    public fieldCentricMovement fieldCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public cameraLibrary cameraLibrary;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;

    public detectedId motif = detectedId.UNKNOWN;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary);

        cameraLibrary.initializeAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive() && motif == detectedId.UNKNOWN) {

                motif = cameraLibrary.detectID();

                if (motif == detectedId.GREEN_PURPLE_PURPLE) {
                    mainLibrary.rgbIndicator.setPosition(.280); //red
                    telemetry.addLine("Works");
                } else if (motif == detectedId.PURPLE_GREEN_PURPLE) {
                    mainLibrary.rgbIndicator.setPosition(.333); //orange
                    telemetry.addLine("Works");
                } else if (motif == detectedId.PURPLE_PURPLE_GREEN) {
                    mainLibrary.rgbIndicator.setPosition(.388); //yellow
                    telemetry.addLine("Works");
                } else {
                    mainLibrary.rgbIndicator.setPosition(0);
                }




            }
        }

        blackboard.put("MOTIF", motif);



    }
}
