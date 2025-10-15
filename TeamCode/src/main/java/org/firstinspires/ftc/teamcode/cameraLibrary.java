package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.List;
import java.util.Objects;

public class cameraLibrary {

    public mainLibrary mainLibrary;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public AprilTagProcessor aprilTag;

    public HardwareMap hwMap;

    public cameraLibrary(OpMode opMode, mainLibrary mainLibrary) {

        this.hwMap = opMode.hardwareMap;

        this.mainLibrary = mainLibrary;

    }

    public enum detectedId {

        GREEN_PURPLE_PURPLE,

        PURPLE_GREEN_PURPLE,

        PURPLE_PURPLE_GREEN,

        UNKNOWN

    }
    public static final boolean useWebcam = true;

    public int aprilTagID_GPP = 21;

    VisionPortal.Builder builder = new VisionPortal.Builder();

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            5, 2, 3.75, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -40, 0, 0);

    public void initializeAprilTag() {

        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();

        if (useWebcam) {
            builder.setCamera(hwMap.get(WebcamName.class, "Logitech Webcam"));
            //visionPortal = VisionPortal.easyCreateWithDefaults(hwMap.get(WebcamName.class, "Logitech Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);

        VisionPortal visionPortal = builder.build();

    }
    public detectedId detectID() {
        detectedId idDetected = detectedId.UNKNOWN;
        List <AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 21) {
                idDetected = detectedId.GREEN_PURPLE_PURPLE;
            } else if (detection.id == 22) {
                idDetected = detectedId.PURPLE_GREEN_PURPLE;
            } else if (detection.id == 23) {
                idDetected = detectedId.PURPLE_PURPLE_GREEN;
            } else {
                mainLibrary.telemetry.addData("Could not detect the motif pattern due to problem scanning april tag", detection.id);
            }
        }
        return idDetected;
    }


    //may detect two tags at once and only return one. please fix later
    public Pose3D tagReferencePosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Pose3D tagOrientation = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    tagOrientation = detection.robotPose;
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return tagOrientation;
    }

    public Pose3D tagReferencePositionFromGoal() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Pose3D tagOrientation = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    tagOrientation = detection.robotPose;
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch,
                            detection.ftcPose.roll,
                            detection.ftcPose.yaw));
                }
            }
            else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return tagOrientation;
    }

    public void cameraTelemetry() {
        List <AprilTagDetection> currentDetections = aprilTag.getDetections();
        mainLibrary.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                mainLibrary.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                mainLibrary.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                mainLibrary.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                mainLibrary.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
    }

}}
