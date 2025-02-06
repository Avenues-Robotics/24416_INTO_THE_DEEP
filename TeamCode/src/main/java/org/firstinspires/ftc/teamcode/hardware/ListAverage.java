package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayDeque;

public class ListAverage {

    int size_limit;
    double sum = 0.0;
    double average = 0.0;
    ArrayDeque<Double> measurements;


    public ListAverage(int size_limit){
        this.size_limit = size_limit;
        ArrayDeque<Double> measurements = new ArrayDeque<>();
    }
    public double listAverage(double newValue) {

        if (measurements.size() > size_limit) {
            double oldest = measurements.getLast();// Remove oldest measurement from the list
            sum = sum - oldest;
        }
        measurements.addFirst(newValue); // Add a new item to the list
        sum = sum + newValue;

        if (measurements.size() > 0) {
            average = sum / measurements.size();
        }
        return average;
    }

    public int getSize(){
        return this.measurements.size();
    }
}

