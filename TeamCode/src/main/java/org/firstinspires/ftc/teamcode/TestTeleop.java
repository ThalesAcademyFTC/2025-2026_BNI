package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.cameraLibrary.detectedId;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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

    public static final String MOTIF = "MOTIF";

    Pose3D robotPosition = null;

    detectedId motif;

    public void init() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        cameraLibrary.initializeAprilTag();

        motif = (detectedId) blackboard.get("MOTIF");


        telemetry.addData("Touch sensor state", mainLibrary.touchSensor.isPressed());
        telemetry.addData("Distance from distance sensor", mainLibrary.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Red value", mainLibrary.colorSensor.red());
        telemetry.addData("Blue value", mainLibrary.colorSensor.blue());
        telemetry.addData("Green value", mainLibrary.colorSensor.green());
        telemetry.addData("Alpha value", mainLibrary.colorSensor.alpha());
        telemetry.addData("ColorRGB", sensorLibrary.isColorRGB());
        telemetry.addData("ColorHSV", sensorLibrary.isColorHSV());
        telemetry.addData("Color sensor distance in cm", mainLibrary.colorSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Is PurpleRGB?", sensorLibrary.IsPurple());
        telemetry.addData("ServoPos value for rgb indicator", ledValue);

    }

    public void loop() {


/*
        if (sensorLibrary.isColorHSV() == org.firstinspires.ftc.teamcode.sensorLibrary.colorValues.GREEN) {
            ledValue = .5;
        }
        if (sensorLibrary.isColorHSV() == org.firstinspires.ftc.teamcode.sensorLibrary.colorValues.PURPLE) {
            ledValue = .722;
        }
*/
        if (motif == detectedId.GREEN_PURPLE_PURPLE) {
            mainLibrary.rgbIndicator.setPosition(.280); //red
        } else if (motif == detectedId.PURPLE_GREEN_PURPLE) {
            mainLibrary.rgbIndicator.setPosition(.333); //orange
        } else if (motif == detectedId.PURPLE_PURPLE_GREEN) {
            mainLibrary.rgbIndicator.setPosition(.388); //yellow
        } else {
            telemetry.addData("April ID", motif);
        }
        //mainLibrary.rgbIndicator.setPosition(ledValue);
        cameraLibrary.cameraTelemetry();
        robotPosition = cameraLibrary.tagReferencePosition();




        telemetry.update();

    }
}
