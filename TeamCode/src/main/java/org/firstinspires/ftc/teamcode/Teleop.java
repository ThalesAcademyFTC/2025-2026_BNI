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

    double rbtSpd = .5;

    double startPosition = .5;

    public void init() {

        mainLibrary = new mainLibrary(hardwareMap, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);

        driverCentricMovement = new driverCentricMovement(mainLibrary);

        fieldCentricMovement = new fieldCentricMovement(mainLibrary);

        mainLibrary = new mainLibrary(this, org.firstinspires.ftc.teamcode.mainLibrary.Drivetrain.MECHANUM);
        IMU imu = mainLibrary.hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



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

        double y = (-gamepad1.left_stick_y);
        double x = (gamepad1.left_stick_x);
        double turn = (gamepad1.right_stick_x);

        //driverCentricMovement.driverMovement(x, y, turn);
        fieldCentricMovement.fieldMovement(x, y, turn);


        telemetry.update();

        if (gamepad2.right_trigger > 0.5) {
            mainLibrary.servo1.setPosition(startPosition += 0.05);
        }
        if (gamepad2.left_trigger > 0.5) {
            mainLibrary.servo1.setPosition(startPosition -= 0.05);
        }
        if (gamepad2.a) {
            if (sensorLibrary.isColor("red")) {
                telemetry.addLine("Red");

            } else if (sensorLibrary.isColor("blue")) {
                telemetry.addLine("Blue");

            } else if (sensorLibrary.isColor("green")) {
                telemetry.addLine("Green");

            } else if (sensorLibrary.isColor("white")) {
                telemetry.addLine("White");

            } else {
                telemetry.addLine("Not Found");

            }
        }
    }
}
