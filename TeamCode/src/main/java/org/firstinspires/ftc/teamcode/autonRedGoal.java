package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class autonRedGoal extends LinearOpMode {

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

    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        //cameraLibrary = new cameraLibrary(this, mainLibrary);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

       // cameraLibrary.initializeAprilTag();


        //start of the auton
        waitForStart();

        movement.moveBackward(50, 0.5);

        //cameraLibrary.detectIfShotPossible();

        sleep(1000);

        movement.launchLittleBoy();

        sleep(1500);

        movement.turnLeft(45, 0.5);

        sleep(1000);

        movement.moveForward(24, 0.5);

        /*mainLibrary.cannonMotor.setPower(1);

        if (getRuntime() >= last_time + 1.5) {
            movement.primeLaunch();

            sleep(100);

            movement.restTHESERVO();

            sleep(100);

            mainLibrary.cannonMotor.setPower(1);

            if (getRuntime() >= last_time + 5) {
                movement.primeLaunch();

                sleep(100);

                movement.restTHESERVO();

                sleep(100);

                mainLibrary.cannonMotor.setPower(1);

                if (getRuntime() >= last_time + 8.5) {
                    movement.primeLaunch();

                    sleep(100);

                    movement.restTHESERVO();

                    sleep(1000);

                    movement.turnRight(45, 0.5);

                    sleep(1000);

                    movement.moveRight(12, 0.5);
                }
            }
        } else {*/

     /*   }

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

*/



    }
}
