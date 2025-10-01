package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class testAuton extends LinearOpMode {

    public driverCentricMovement driverCentricMovement;

    public fieldCentricMovement fieldCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public cameraLibrary cameraLibrary;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary);

        cameraLibrary.initializeAprilTag();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (cameraLibrary.detectID() == org.firstinspires.ftc.teamcode.cameraLibrary.detectedId.GREEN_PURPLE_PURPLE) {
                    mainLibrary.rgbIndicator.setPosition(.277); //red
                    telemetry.addLine("Works");
                } else if (cameraLibrary.detectID() == org.firstinspires.ftc.teamcode.cameraLibrary.detectedId.PURPLE_GREEN_PURPLE) {
                    mainLibrary.rgbIndicator.setPosition(.333); //orange
                    telemetry.addLine("Works");
                } else if (cameraLibrary.detectID() == org.firstinspires.ftc.teamcode.cameraLibrary.detectedId.PURPLE_PURPLE_GREEN) {
                    mainLibrary.rgbIndicator.setPosition(.388); //yellow
                    telemetry.addLine("Works");
                } else if (cameraLibrary.detectID() == org.firstinspires.ftc.teamcode.cameraLibrary.detectedId.UNKNOWN) {
                    mainLibrary.rgbIndicator.setPosition(.444); //sage green
                    telemetry.addLine("Works");
                } else {
                    mainLibrary.rgbIndicator.setPosition(0);
                }




            }
        }



    }
}
