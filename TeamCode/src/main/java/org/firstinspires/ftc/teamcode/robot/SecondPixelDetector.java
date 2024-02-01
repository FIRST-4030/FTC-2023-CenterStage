package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SecondPixelDetector {
    DistanceSensor pixelSensor;
    double distAverage;
    double currentDist;
    public int arraySize = 3;
    double[] distArray = new double[arraySize];
    int distIterator = 0;

    public SecondPixelDetector(HardwareMap hardwareMap){
        pixelSensor = hardwareMap.get(DistanceSensor.class, "detector");
    }
    public void shiftRunningAverage(int num){

    }
    public double update(){
        currentDist = pixelSensor.getDistance(MM);
        distArray[distIterator % (arraySize-1)] = currentDist;
        distIterator++;
        for(int i = 0; i < distArray.length; i++){
            distAverage += distArray[i];
        }
        distAverage /= (arraySize);
        return distAverage;
    }

    public double getCurrentDist(){
        currentDist = pixelSensor.getDistance(MM);
        return currentDist;
    }
}
