package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    public movement movement;

    public detectedId motif = detectedId.UNKNOWN;

    public double distanceForShot; //needs a value


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary);

        movement = new movement(mainLibrary, driverCentricMovement);

        cameraLibrary.initializeAprilTag();

        waitForStart();

        while(opModeIsActive() && cameraLibrary.detectIfShotPossible()) {

            /*while (cameraLibrary.detectID() == detectedId.BLUE_GOAL) {

                mainLibrary.rgbIndicator.setPosition(0.611);

            }

             */

          /*  movement.moveForward(50, 0.3);
            sleep(1000);
            movement.moveRight(50, 0.3);
            sleep(1000);
            movement.moveBackward(50, 0.3);
            sleep(1000);
            movement.moveLeft(50, 0.3);
*/


            //cameraLibrary.detectIfShotPossible().getOrientation().getPitch(AngleUnit.RADIANS);
            //cameraLibrary.detectIfShotPossible().getOrientation().getRoll(AngleUnit.RADIANS);
            //cameraLibrary.detectIfShotPossible().getOrientation().getYaw(AngleUnit.RADIANS);


            /*if (cameraLibrary.tagReferencePositionFromGoal() == cameraLibrary.detectIfShotPossible()) {

                movement.turnLeft(5000, .1);

            }
*/
            // movement.moveForward(3, .5);
            // movement.moveBackward(3, .5);
            /*while(opModeIsActive()) { //&& distance from april tag
            // != distanceForShot) {

                while (cameraLibrary.detectID() != detectedId.BLUE_GOAL) {

                    movement.turnLeft(360, 0.1);

                }

                while (mainLibrary.rgbIndicator.getPosition() != .278) {

                    cameraLibrary.detectIfShotPossible();

                }

                movement.moveForward(3, .5);
                movement.moveBackward(3, .5);

            }
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
            */

        }



    }
}
