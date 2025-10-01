package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTagProcessor;

    double rbtSpd = .5;

    double startPosition = .5;

    double ledValue = 0.277;

    double constant = 1;

    public void init() {

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary);

        cameraLibrary.initializeAprilTag();

    }
    public void loop() {

        cameraLibrary.detectID();

        if (sensorLibrary.isColorHSV() == org.firstinspires.ftc.teamcode.sensorLibrary.colorValues.GREEN) {
            ledValue = .5;
        } else if (sensorLibrary.isColorHSV() == org.firstinspires.ftc.teamcode.sensorLibrary.colorValues.PURPLE) {
            ledValue = .722;
        } else {
            ledValue = 0;
        }

        mainLibrary.rgbIndicator.setPosition(ledValue);
        cameraLibrary.cameraTelemetry();
        //telemetry.addData("Touch sensor state", mainLibrary.touchSensor.isPressed());
        //telemetry.addData("Distance from distance sensor", mainLibrary.distanceSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Red value", mainLibrary.colorSensor.red());
        //telemetry.addData("Blue value", mainLibrary.colorSensor.blue());
        //telemetry.addData("Green value", mainLibrary.colorSensor.green());
        //telemetry.addData("Alpha value", mainLibrary.colorSensor.alpha());
        // telemetry.addData("ColorRGB", sensorLibrary.isColorRGB());
        //telemetry.addData("ColorHSV", sensorLibrary.isColorHSV());
        //telemetry.addData("Color sensor distance in cm", mainLibrary.colorSensor.getDistance(DistanceUnit.CM));
        // telemetry.addData("Is PurpleRGB?", sensorLibrary.IsPurple());
        //telemetry.addData("ServoPos value for rgb indicator", ledValue);
        telemetry.update();

    }

}
