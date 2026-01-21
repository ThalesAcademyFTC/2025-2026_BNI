package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class cameraLibrary {

    public mainLibrary mainLibrary;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public driverCentricMovement driverCentricMovement;

    public AprilTagProcessor aprilTag;

    public HardwareMap hwMap;

    public movement movement;

    public OpMode opmode;

    public LinearOpMode auton;

    public cameraLibrary(OpMode opMode, mainLibrary mainLibrary, movement movement, driverCentricMovement driverCentricMovement) {

        this.hwMap = opMode.hardwareMap;

        this.mainLibrary = mainLibrary;

        this.driverCentricMovement = driverCentricMovement;

        this.opmode = opMode;

    }
    public cameraLibrary(LinearOpMode opMode, mainLibrary mainLibrary, movement movement, driverCentricMovement driverCentricMovement) {

        this.hwMap = opMode.hardwareMap;

        this.mainLibrary = mainLibrary;

        this.driverCentricMovement = driverCentricMovement;

        this.opmode = opMode;

    }



    public enum detectedId {

        BLUE_GOAL,

        GREEN_PURPLE_PURPLE,

        PURPLE_GREEN_PURPLE,

        PURPLE_PURPLE_GREEN,

        RED_GOAL,

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
            } else if (detection.id == 20) {
                idDetected = detectedId.BLUE_GOAL;
            } else if (detection.id == 24) {
                idDetected = detectedId.RED_GOAL;
            } else {
                mainLibrary.telemetry.addData("Could not detect the motif pattern due to problem scanning april tag", detection.id);
            }
        }
        return idDetected;
    }

    public double yawAccordingToAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double yawValue = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (!detection.metadata.name.contains("Obelisk")) {
                    yawValue = detection.robotPose.getOrientation().getYaw();
                }
            }
        } return yawValue;


    }

    //may detect two tags at once and only return one. please fix later
    public Pose3D tagReferencePosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Pose3D tagOrientation = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    tagOrientation = detection.robotPose;
                    mainLibrary.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    mainLibrary.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            }
            else {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                mainLibrary.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return tagOrientation;
    }

    public AprilTagPoseFtc tagReferencePositionFromGoal() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagPoseFtc tagOrientation = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    tagOrientation = detection.ftcPose;
                    mainLibrary.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    mainLibrary.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch,
                            detection.ftcPose.roll,
                            detection.ftcPose.yaw));
                }
            }
            else {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                mainLibrary.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return tagOrientation;
    }

    public boolean moveY(double current, double desiredY) {
        if (current >= desiredY - 2 && current <= desiredY + 2) {

            return true;

        } else if (current > desiredY) {

            driverCentricMovement.driverMovement(0, .2, 0);

        } else {

            driverCentricMovement.driverMovement(0, -.2, 0);

        }
        return false;
    }

    public boolean moveX(double current, double desiredX) {
        if (current >= desiredX - 2 && current <= desiredX + 2) {

            return true;

        } else if (current > desiredX) {

            driverCentricMovement.driverMovement(.2, 0, 0);

        } else {

            driverCentricMovement.driverMovement(-.2, 0, 0);

        }
        return false;
    }

    public boolean moveYaw(double current, double desiredYaw) {
        if (current >= desiredYaw - 4 && current <= desiredYaw + 4) {

            return true;

        } else {

            driverCentricMovement.driverMovement(0,0,-.2);

        }
        return false;
    }
   /* public boolean moveToDesiredGoalPos() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                // Only use tags that don't have Obelisk in them
                if (detection.metadata.name.contains("Goal")) {
                    mainLibrary.tagOrientationX = detection.ftcPose.x;
                    mainLibrary.tagOrientationY = detection.ftcPose.y;
                    mainLibrary.tagOrientationZ = detection.ftcPose.z;
                    mainLibrary.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z));
                    mainLibrary.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.ftcPose.pitch,
                            detection.ftcPose.roll,
                            detection.ftcPose.yaw));
                }
                moveY(mainLibrary.tagOrientationY, desiredY);



            } else {
                mainLibrary.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                mainLibrary.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        return false;
    }


*/
    public boolean detectIfShotPossible() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        Pose3D tagOrientation = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                tagOrientation = detection.robotPose;
                mainLibrary.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.z));
                mainLibrary.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.yaw));
                if (detection.ftcPose.y >= 50 && detection.ftcPose.y <= 65) {

                    mainLibrary.shotPossibility = true;

                } else {

                    mainLibrary.shotPossibility = false;

                }

            }
        } return mainLibrary.shotPossibility;
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

}

    public void autoPositionGoal(double x, double y, double yaw) {
        AprilTagPoseFtc pose;
        AprilTagPoseFtc pose1;
        AprilTagPoseFtc pose2;

        boolean inPositionY = false;
        boolean inPositionX = false;
        boolean inPositionZ = false;

        while (!inPositionZ && !auton.isStopRequested()) {
            pose2 = tagReferencePositionFromGoal();
            if (pose2 == null) {
                mainLibrary.telemetry.addData("No april tag found :(", pose2);
                mainLibrary.telemetry.update();
            } else {
                inPositionZ = moveYaw(pose2.yaw, yaw);
                mainLibrary.telemetry.addLine(String.format("Tag found tracking X position %6.1f / %6.1f", pose2.yaw, yaw));
                mainLibrary.telemetry.update();
            }
        }
        while (!inPositionY && !auton.isStopRequested()) {
            pose = tagReferencePositionFromGoal();
            if (pose == null) {
                mainLibrary.telemetry.addData("No april tag found :(", pose);
                mainLibrary.telemetry.update();
            } else {
                inPositionY = moveY(pose.y, y);
                mainLibrary.telemetry.addLine(String.format("Tag found tracking Y position %6.1f / %6.1f", pose.y, y));
                mainLibrary.telemetry.update();
            }
        }
        while (!inPositionX && !auton.isStopRequested()) {
            pose1 = tagReferencePositionFromGoal();
            if (pose1 == null) {
                mainLibrary.telemetry.addData("No april tag found :(", pose1);
                mainLibrary.telemetry.update();
            } else {
                inPositionX = moveX(pose1.x, x);
                mainLibrary.telemetry.addLine(String.format("Tag found tracking X position %6.1f / %6.1f", pose1.x, x));
                mainLibrary.telemetry.update();
            }
        }
    }
}
