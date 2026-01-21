package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.system.Os;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Teleop extends OpMode {
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

    double power = .95;

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
        telemetry.addData("Motor Speed", () -> { return mainLibrary.cannonMotor1.getPower();});
        telemetry.addData("Motor 2 Speed", () -> { return mainLibrary.cannonMotor2.getPower();});
    }
    public void loop() {

        telemetry.update();


        //sets the move function to controller values
        double y = (-gamepad1.left_stick_y);
        double x = (-gamepad1.left_stick_x);
        double turn = (gamepad1.right_stick_x);


        driverCentricMovement.driverMovement(x, y, turn);
        //fieldCentricMovement.fieldMovement(x, y, turn);

        if (gamepad2.left_bumper && mainLibrary.leftToggleToggle) {

            movement.primeLaunch();

            mainLibrary.leftToggleToggle = false;

        } else if ((gamepad2.right_bumper && !mainLibrary.leftToggleToggle)){

            movement.restTHESERVO();

            mainLibrary.leftToggleToggle = true;

        }

        if (gamepad2.dpadDownWasPressed() && power != 0) {
            power -= 0.05;
        }
        if (gamepad2.dpadUpWasPressed() && power != 1) {
            power += 0.05;
        }

        if (gamepad2.left_trigger >= 0.5) {

            movement.cannonLaunch(power);

        } else if (gamepad2.right_trigger >= 0.5) {
            movement.cannonLaunch(power);

            if (last_time == 0.0) {
                last_time = getRuntime();
            }

            curr_time = getRuntime();
            if (curr_time > (last_time + 1.5)) {
                movement.primeLaunch();
            }

            if (curr_time > (last_time + 2)) {
                movement.restTHESERVO();
                last_time = curr_time;
            }

        } else if (gamepad2.right_trigger < .5 && gamepad2.left_trigger < .5) {
            last_time = 0.0;
            movement.cannonStop();
        }
    }
}
