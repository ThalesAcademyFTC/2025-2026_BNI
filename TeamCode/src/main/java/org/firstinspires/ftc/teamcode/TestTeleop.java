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

        telemetry.update();

        /*
        experimental blue goal teleop function that adds aim mode to the teleop controller
        allowing for the camera to scan the april tag and constantly adjust and turn to reach
        the ideal shooting position while still allowing fblr movement for manual shooting
        adjustments.
        */

        pose = cameraLibrary.tagReferencePositionFromBlue();

        if (gamepad1.right_trigger > .5) {

            driverCentricMovement.driverMovement(gamepad1.left_stick_x, gamepad1.left_stick_y, turn);
            THEBOOZE = cameraLibrary.moveBearing(cameraLibrary.FINAL_BLUE_BEARING, pose.yaw);

            if (THEBOOZE) {

                turn = 0;
                THEBOOZE = false;

            }
        } else {

            driverCentricMovement.driverMovement(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            turn = .5;

        }

    }
}
