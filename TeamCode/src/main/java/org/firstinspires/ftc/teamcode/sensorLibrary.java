package org.firstinspires.ftc.teamcode;


//import com.qualcomm.robotcore.hardware.IMU;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class sensorLibrary {

    public mainLibrary mainLibrary;

    public sensorLibrary(mainLibrary mainLibrary) {

        this.mainLibrary = mainLibrary;

    }

    public boolean isColor(String color) {
        int red = mainLibrary.colorSensor.red();
        int blue = mainLibrary.colorSensor.blue();
        int green = mainLibrary.colorSensor.green();
        int alpha = mainLibrary.colorSensor.alpha();

        switch (color.toLowerCase()) {
            case "red":
                return red > green && red > blue;
            case "blue":
                return blue > green && blue > red;
            case "green":
                return green > blue && green > red;
            case "white":
                return alpha >= 1000;
            default:
                return false;
        }
    }
    public String detectColor() {
        String color = "Cannot detect";
        if (isColor("red")) {

            return color = "red";

        }
        if (isColor("blue")) {

            return color = "blue";

        }
        if (isColor("green")) {

            return color = "green";

        }
        if (isColor("white")) {

            return color = "white";

        }
        return color;
    }

   public boolean touchSensorState() {
        boolean state;
        return state = mainLibrary.touchSensor.isPressed();
    }

    public double distanceFromSensor() {
        //CM for it being a 2m distance sensor, INCHES for correlating to tick to inches
        return mainLibrary.distanceSensor.getDistance(DistanceUnit.CM);
    }
//not happy



}

