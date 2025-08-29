package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.system.Os;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class Teleop extends OpMode {
    public driverCentricMovement driverCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public void init() {

        mainLibrary = new mainLibrary(hardwareMap, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        sensorLibrary = new sensorLibrary(mainLibrary);

        telemetry.addData("Touch sensor state", mainLibrary.touchSensor.isPressed());
        telemetry.addData("Distance from distance sensor", mainLibrary.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Red value", mainLibrary.colorSensor.red());
        telemetry.addData("Blue value", mainLibrary.colorSensor.blue());
        telemetry.addData("Green value", mainLibrary.colorSensor.green());
        telemetry.addData("Alpha value", mainLibrary.colorSensor.alpha());
        telemetry.addData("Color", sensorLibrary.detectColor());
    }
    public void loop() {

        driverCentricMovement.movement(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.update();
    }
}
