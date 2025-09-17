package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestTeleop extends OpMode {

    public driverCentricMovement driverCentricMovement;

    public fieldCentricMovement fieldCentricMovement;

    public mainLibrary mainLibrary;

    public sensorLibrary sensorLibrary;

    double rbtSpd = .5;

    double startPosition = .5;

    double constant = 1;

    public void init() {

        mainLibrary = new mainLibrary(hardwareMap, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        sensorLibrary = new sensorLibrary(mainLibrary);

    }
    public void loop() {

        while (constant == 1) {
            telemetry.addData("Touch sensor state", mainLibrary.touchSensor.isPressed());
            telemetry.addData("Distance from distance sensor", mainLibrary.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Red value", mainLibrary.colorSensor.red());
            telemetry.addData("Blue value", mainLibrary.colorSensor.blue());
            telemetry.addData("Green value", mainLibrary.colorSensor.green());
            telemetry.addData("Alpha value", mainLibrary.colorSensor.alpha());
            telemetry.addData("Color", sensorLibrary.detectColor());
            telemetry.update();
        }
    }

}
