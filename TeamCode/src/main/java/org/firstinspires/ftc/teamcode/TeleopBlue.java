package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp
public class TeleopBlue extends OpMode {
    public driverCentricMovement driverCentricMovement;

    public fieldCentricMovement fieldCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public movement movement;

    public cameraLibrary cameraLibrary;

    double rbtSpd = .5;

    double startPosition = .5;

    double rgbPos = .277;

    double keep = 0;

    double start = 0;

    double curr_time;

    double last_time;

    double power = .84;

    double times = 0;

    double turn = .25;

    boolean THEBOOZE = true;

    public AprilTagPoseFtc pose;

    double BEARING_ADJUSTMENT = -5;

    boolean autoAimActive = false;

    public void init() {
        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        cameraLibrary = new cameraLibrary(this, mainLibrary, movement, driverCentricMovement);

        //start pos 0.7
        movement.restTHESERVO();

        sensorLibrary = new sensorLibrary(mainLibrary);
        //mainLibrary.explode();

        //telemetry.addData("Motif pattern", blackboard.get())

        //telemetry.addData("Touch sensor state", mainLibrary.touchSensor.isPressed());
        //telemetry.addData("Distance from distance sensor", mainLibrary.distanceSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Red value", mainLibrary.colorSensor.red());
        //telemetry.addData("Blue value", mainLibrary.colorSensor.blue());
        //telemetry.addData("Green value", mainLibrary.colorSensor.green());
        //telemetry.addData("Alpha value", mainLibrary.colorSensor.alpha());
        //telemetry.addData("Color", sensorLibrary.isColorRGB());
        telemetry.addData("Motif Pattern is", blackboard.get(mainLibrary.motifPattern));
        telemetry.addData("Motor Speed", () -> { return power;});
        telemetry.addData("Times (experimental)", () -> { return times;});

        mainLibrary.rgbIndicator.setPosition(.277);

        cameraLibrary.initializeAprilTag();
    }
    public void loop() {

        telemetry.update();

        pose = cameraLibrary.tagReferencePositionFromBlue();

        if (pose != null) {
            //is detected
            if (pose.bearing >= -10 && pose.bearing <= 0) {
                //in right pos
                mainLibrary.rgbIndicator.setPosition(.5);
            } else {
                mainLibrary.rgbIndicator.setPosition(.611);
            }
        } else {
            //not detected
            mainLibrary.rgbIndicator.setPosition(.278);
        }

        double x = (gamepad1.left_stick_x*.75);
        double y = (-gamepad1.left_stick_y*.75);
        double turn = (gamepad1.right_stick_x*.75);

        /*if (gamepad1.a) {
            if (pose != null) {
                pose = cameraLibrary.tagReferencePositionFromBlue();
                if (pose != null) {
                    double bearing = pose.bearing;
                    mainLibrary.rgbIndicator.setPosition(.722);
                    autoAimActive = true;
                    double initialDirection = 0; // get imu yaw
                    turn = bearing > 0 ? 0.2 : -0.2;
                } else {
                    mainLibrary.rgbIndicator.setPosition(0);  // april tag not found
                }
            }
            if(autoAimActive) {
                    double currentDirection; // get from imu

                }
            }
        } else {
            pose = null;
            autoAimActive = false;
            mainLibrary.rgbIndicator.setPosition(0);
        }
*/
        driverCentricMovement.driverMovement(x, y, turn);

        //sets the move function to controller values

        if (gamepad2.left_bumper && mainLibrary.leftToggleToggle) {

            movement.primeLaunch();

            mainLibrary.leftToggleToggle = false;

        } else if ((gamepad2.right_bumper && !mainLibrary.leftToggleToggle)){

            movement.restTHESERVO();

            mainLibrary.leftToggleToggle = true;

        }

        if (gamepad2.dpadDownWasPressed() && power != 0) {
            power -= 0.01;
        }
        if (gamepad2.dpadUpWasPressed() && power != 1) {
            power += 0.01;
        }

        if (gamepad2.left_trigger >= 0.5) {

            movement.cannonLaunch(power);

        } else if (gamepad2.right_trigger >= 0.5) {
            movement.cannonLaunch(power);

            if (last_time == 0.0) {
                last_time = getRuntime();
            }

            curr_time = getRuntime();
            if (curr_time > (last_time + 2)) {
                movement.primeLaunch();
            }
            if (curr_time > (last_time + 3)) {
                movement.restTHESERVO();
                last_time = curr_time;
            }

        } else if (gamepad2.right_trigger < .5 && gamepad2.left_trigger < .5) {
            last_time = 0.0;
            movement.cannonStop();
            movement.restTHESERVO();
            times = 0;
        }
    }
}
