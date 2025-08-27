package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    public driverCentricMovement driverCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    public void init() {
        telemetry.addData("Touch sensor state", mainLibrary.touchSensor.isPressed());
        telemetry.addData("Distance from distance sensor", sensorLibrary.distanceFromSensor());
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
