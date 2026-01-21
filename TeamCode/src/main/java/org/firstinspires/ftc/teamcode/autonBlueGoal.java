package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
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

    public boolean inPositionY = false;

    public boolean inPositionX = false;

    public boolean inPositionZ = false;

    double speed = 0.5;

    double power = .85;

    double X = -7.6;

    final double Y = 61;

    final double YAW = -13;


    public void runOpMode() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        cameraLibrary.initializeAprilTag();

        //start of the auton
        waitForStart();

        AprilTagPoseFtc pose;
        AprilTagPoseFtc pose1;
        AprilTagPoseFtc pose2;

        movement.moveBackward(48, speed);

        sleep(200);

        cameraLibrary.autoPositionGoal(X, Y, YAW);

        /*while (!inPositionZ && !isStopRequested()) {
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
        while (!inPositionY && !isStopRequested()) {
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
        while (!inPositionX && !isStopRequested()) {
            pose1 = cameraLibrary.tagReferencePositionFromGoal();
            if (pose1 == null) {
                telemetry.addData("No april tag found :(", pose1);
                telemetry.update();
            } else {
                inPositionX = cameraLibrary.moveX(pose1.x, cameraLibrary.DESIRED_X);
                mainLibrary.telemetry.addLine(String.format("Tag found tracking X position %6.1f / %6.1f", pose1.x, cameraLibrary.DESIRED_X));
                telemetry.update();
            }
        }*/

        movement.launchLittleBoy(power);

        sleep(500);

        movement.turnLeft(45, speed);

        sleep(200);

        movement.moveRight(24,speed);





    }
}
