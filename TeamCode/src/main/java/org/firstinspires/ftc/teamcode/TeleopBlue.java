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

    double power = .87;

    double times = 0;

    double turn = .25;

    boolean THEBOOZE = true;

    public AprilTagPoseFtc pose;

    public void init() {
        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        movement = new movement(mainLibrary, driverCentricMovement, cameraLibrary);

        //start pos 0.7
        movement.restTHESERVO();

        sensorLibrary = new sensorLibrary(mainLibrary);

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
    }
    public void loop() {

        telemetry.update();


        //sets the move function to controller values

        pose = cameraLibrary.tagReferencePositionFromBlue();

        if (gamepad1.right_trigger > .5) {

            driverCentricMovement.driverMovement(gamepad1.left_stick_x, gamepad1.left_stick_y, turn);
            THEBOOZE = cameraLibrary.moveYaw(cameraLibrary.FINAL_BLUE_YAW, pose.yaw);

            if (THEBOOZE) {

                turn = 0;
                THEBOOZE = false;

            }
        } else {

            driverCentricMovement.driverMovement(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            turn = .25;

        }

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
            if (curr_time > (last_time + 2 ) && times == 0) {
                movement.primeLaunch();
                times += 1;
            }

            if (curr_time > (last_time + 1 ) && times > 0 && times <= 3) {
                movement.primeLaunch();
                times += 1;
            } else if (times > 3) {
                times = 0;
            }

            if (curr_time > (last_time + 2.1)) {
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
