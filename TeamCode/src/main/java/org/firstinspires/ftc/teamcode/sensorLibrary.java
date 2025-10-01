package org.firstinspires.ftc.teamcode;


//import com.qualcomm.robotcore.hardware.IMU;


import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.Objects;


public class sensorLibrary {

    public mainLibrary mainLibrary;

    public ConceptAprilTagEasy conceptAprilTagEasy;

    public enum colorValues {
        WHITE,

        PURPLE,

        GREEN,

        CANNOT_DETECT,

        UNKNOWN
    }

    public colorValues colors;

    public sensorLibrary(mainLibrary mainLibrary) {

        this.mainLibrary = mainLibrary;

    }
    public colorValues isColorHSV() {
        //purple value 278°, 93%, 96
        //green value 125°, 81%, 54%
        final float[] hsvValues = new float[3];
        NormalizedRGBA normalColors = mainLibrary.colorSensor.getNormalizedColors();
        Color.colorToHSV(normalColors.toColor(), hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];
        float alpha = mainLibrary.colorSensor.alpha();

        colorValues colorHSV = colorValues.UNKNOWN;

        if ((hue >= 200 && hue <= 300) && (saturation == 1 || (saturation >= .6 && saturation <= 1)) && (value >= .002 && value <= .004)) {

            colorHSV = colorValues.PURPLE;

        } else if ((hue >= 100 && hue <= 190) && (saturation == 1 || (saturation >= .6 && saturation <= 1)) && (value >= .004 && value <= .016)) {

            colorHSV = colorValues.GREEN;

        }
        mainLibrary.telemetry.addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        return colorHSV;
    }

    public colorValues isColorRGB() {
        NormalizedRGBA rgba = mainLibrary.colorSensor.getNormalizedColors();
        //float red = rgba.red;
        int red = mainLibrary.colorSensor.red();
        int blue = mainLibrary.colorSensor.blue();
        int green = mainLibrary.colorSensor.green();
        int alpha = mainLibrary.colorSensor.alpha();

        /*switch (color.toLowerCase()) {
            case "white":
                return alpha >= 150;
            case "green":
                return green > blue && green > red && green >= 110;
            case "purple":
                return (red + blue) >= 175 && distanceFromSensor() > 17;
            case "unknown":
                return (red < 90) || (blue < 90) || (green < 110);
            default:
                return false;
        } */
        colorValues color = colorValues.UNKNOWN;

                if ((red + blue) >= 175) {

                    color = colorValues.PURPLE;

                } else if (alpha >= 150) {

                    color = colorValues.WHITE;

                } else if (green > blue && green > red && green >= 110) {

                    color = colorValues.GREEN;

                } else if (red > 90 || blue < 90 || green < 110) {

                    color = colorValues.CANNOT_DETECT;

                }

        return color;
        }

    public String IsPurple() {
        if (isColorRGB() == colorValues.PURPLE) {

            return "is purple";

        } else {

            return "is not purple";

        }


    }
  /*  public String detectColor() {
        String color = "Cannot detect";
        if (isColor("purple")) {

            return color = "purple";

        }
        if (isColor("green")) {

            return color = "green";

        }
        if (isColor("white")) {

            return color = "white";

        }
        if (isColor("unknown")) {

            return color = "values too low";

        }
        return color;
    }
*/
    /*public colorValues() {
        //NormalizedRGBA rgba = mainLibrary.colorSensor.getNormalizedColors();
        //float red = rgba.red;

        int redValue = mainLibrary.colorSensor.red();
        int blueValue = mainLibrary.colorSensor.blue();
        int greenValue = mainLibrary.colorSensor.green();
        int alphaValue = mainLibrary.colorSensor.alpha();

        //[] redValue, blueValue, greenValue, alphaValue;

        return */



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

