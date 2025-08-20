package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ContollerTestTeleop {

    public mainLibrary mainLibrary;

    boolean leftBumper = false;
    boolean rightBumper = false;
    boolean a = false;
    boolean b = false;
    boolean x = false;
    boolean y = false;
    boolean dpadUp = false;
    boolean dpadDown = false;
    boolean dpadLeft = false;
    boolean dpadRight = false;

    int leftStickX = 0;
    int leftStickY = 0;
    int rightStickX = 0;
    int rightStickY = 0;
    int leftTrigger = 0;
    int rightTrigger = 0;


    public void init() {
    mainLibrary.telemetry.addData("leftBumper: ", leftBumper);
    mainLibrary.telemetry.addData("rightBumper: ", rightBumper);
    mainLibrary.telemetry.addData("leftTrigger: ", leftTrigger);
    mainLibrary.telemetry.addData("rightTrigger: ", rightTrigger);
    mainLibrary.telemetry.addData("a: ", a);
    mainLibrary.telemetry.addData("b: ", b);
    mainLibrary.telemetry.addData("x: ", x);
    mainLibrary.telemetry.addData("y: ", y);
    mainLibrary.telemetry.addData("dpadUp: ", dpadUp);
    mainLibrary.telemetry.addData("dpadDown: ", dpadDown);
    mainLibrary.telemetry.addData("dpadLeft: ", dpadLeft);
    mainLibrary.telemetry.addData("dpadRight: ", dpadRight);
    mainLibrary.telemetry.addData("leftStickX: ", leftStickX);
    mainLibrary.telemetry.addData("rightStickY: ", leftStickY);
    mainLibrary.telemetry.addData("leftStickX: ", rightStickX);
    mainLibrary.telemetry.addData("rightStickY: ", rightStickY);

    }

    public void loop() {

    leftStickX = (int) gamepad1.left_stick_x;
    leftStickY = (int) gamepad1.left_stick_y;
    rightStickX = (int) gamepad1.right_stick_x;
    rightStickY = (int) gamepad1.right_stick_y;
    leftTrigger = (int) gamepad1.left_trigger;
    rightTrigger = (int) gamepad1.right_trigger;


    if (gamepad1.left_bumper) { leftBumper = true; }
    else { leftBumper = false; }
    if (gamepad1.right_bumper) { leftBumper = true; }
    else { rightBumper = false; }
    if (gamepad1.a) { a = true; }
    else { a = false; }
    if (gamepad1.b) { b = true; }
    else { b = false; }
    if (gamepad1.x) { x = true; }
    else { x = false; }
    if (gamepad1.y) { y = true; }
    else { y = false; }
    if (gamepad1.dpad_up) { dpadUp = true; }
    else { a = false; }
    if (gamepad1.dpad_up) { dpadUp = true; }
    else { b = false; }
    if (gamepad1.dpad_up) { dpadUp = true; }
    else { x = false; }
    if (gamepad1.dpad_up) { dpadUp = true; }
    else { y = false; }


    }

}
