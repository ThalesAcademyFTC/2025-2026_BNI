package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Timer;

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

    double suspensionPower1 = 1;

    double suspensionPower2 = 1;

    boolean THEBOOZE = true;

    public AprilTagPoseFtc pose;

    double BEARING_ADJUSTMENT = -5;

    boolean autoAimActive = true;

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
            if (pose.bearing >= -10 && pose.bearing <= 10) {
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

        if (gamepad1.a) {
            //start auto aim process
                if (pose != null && autoAimActive) {
                    double power = .3;
                    double bearing = pose.bearing;
                    mainLibrary.rgbIndicator.setPosition(.722);
                    if (bearing <= 10 && bearing >= -10) {
                        power = .13;
                    }
                    //double initialDirection = mainLibrary.imu.getRobotYawPitchRollAngles().getYaw(); // get imu yaw
                    turn = bearing > 0 ? -power : power;
                    driverCentricMovement.driverMovement(x,y,turn);
                    if (bearing <= 7.5 && bearing >= -7.5) {
                        autoAimActive = false;
                    }

                } else {
                    mainLibrary.rgbIndicator.setPosition(0);  // april tag not found
                }
        }

        if (!autoAimActive) {

            double currentTime = movement.timer.milliseconds();

            if (currentTime >= 500) {
                autoAimActive = true;
                currentTime = 0;
            }

        }

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

       /* if (gamepad2.left_trigger >= 0.5) {

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
*/

        if (gamepad1.right_trigger > 0.2) {

            mainLibrary.suspensionMotor1.setPower(-suspensionPower1);
            mainLibrary.suspensionMotor2.setPower(-suspensionPower2);

            if (mainLibrary.suspensionDistance1.getDistance(DistanceUnit.MM) > mainLibrary.suspensionDistance2.getDistance(DistanceUnit.MM)) {

                suspensionPower1 -= .1;
                suspensionPower2 = 1;

            } else if (mainLibrary.suspensionDistance2.getDistance(DistanceUnit.MM) > mainLibrary.suspensionDistance1.getDistance(DistanceUnit.MM)) {

                suspensionPower2 -= .1;
                suspensionPower1 = 1;

            } else {

                suspensionPower2 = 1;
                suspensionPower1 = 1;

            }
        } else if (gamepad1.left_trigger > 0.2) {
            mainLibrary.suspensionMotor1.setPower(suspensionPower1);
            mainLibrary.suspensionMotor2.setPower(suspensionPower2);

            if (mainLibrary.suspensionDistance1.getDistance(DistanceUnit.MM) > mainLibrary.suspensionDistance2.getDistance(DistanceUnit.MM)) {

                suspensionPower1 -= .1;
                suspensionPower2 = 1;

            } else if (mainLibrary.suspensionDistance2.getDistance(DistanceUnit.MM) > mainLibrary.suspensionDistance1.getDistance(DistanceUnit.MM)) {

                suspensionPower2 -= .1;
                suspensionPower1 = 1;

            } else {

                suspensionPower2 = 1;
                suspensionPower1 = 1;

            }
        } else {
            mainLibrary.suspensionMotor1.setPower(0);
            mainLibrary.suspensionMotor2.setPower(0);
            suspensionPower1 = 1;
            suspensionPower2 = 1;
        }
    }
}
