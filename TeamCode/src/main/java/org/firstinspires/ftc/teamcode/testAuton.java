package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
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

    public boolean inPositionY = false;

    public boolean inPositionX = false;

    public boolean inPositionZ = false;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        cameraLibrary.initializeAprilTag();

        waitForStart();

        AprilTagPoseFtc pose;
        AprilTagPoseFtc pose1;
        AprilTagPoseFtc pose2;


        while (!inPositionZ) {
            pose2 = cameraLibrary.tagReferencePositionFromGoal();
            if (pose2 == null) {
                telemetry.addData("No april tag found :(", pose2);
                telemetry.update();
            } else {
                inPositionZ = cameraLibrary.moveYaw(pose2.yaw, cameraLibrary.DESIRED_YAW);
                mainLibrary.telemetry.addLine(String.format("Tag found tracking X position %6.1f / %6.1f", pose2.yaw, cameraLibrary.DESIRED_YAW));
                telemetry.update();
            }
        }
        while (!inPositionY) {
            pose = cameraLibrary.tagReferencePositionFromGoal();
            if (pose == null) {
                    telemetry.addData("No april tag found :(", pose);
                    telemetry.update();
            } else {
                    inPositionY = cameraLibrary.moveY(pose.y, cameraLibrary.DESIRED_Y);
                    mainLibrary.telemetry.addLine(String.format("Tag found tracking Y position %6.1f / %6.1f", pose.y, cameraLibrary.DESIRED_Y));
                    telemetry.update();
            }
        }
        while (!inPositionX) {
            pose1 = cameraLibrary.tagReferencePositionFromGoal();
            if (pose1 == null) {
                telemetry.addData("No april tag found :(", pose1);
                telemetry.update();
            } else {
                inPositionX = cameraLibrary.moveX(pose1.x, cameraLibrary.DESIRED_X);
                mainLibrary.telemetry.addLine(String.format("Tag found tracking X position %6.1f / %6.1f", pose1.x, cameraLibrary.DESIRED_X));
                telemetry.update();
            }
        }


            /*cameraLibrary.detectIfShotPossible();

            telemetry.update();

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

