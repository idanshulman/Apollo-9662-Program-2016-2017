package org.firstinspires.ftc.teamcode;

public class StopWatch_New {

    private long startTime = 0;
    private long stopTime = 0;
    private boolean running = false;


    public void start() {
        if(!running) {
            this.startTime = System.currentTimeMillis();
        }
        this.running = true;
    }


    public void stop() {
        this.stopTime = System.currentTimeMillis();
        this.running = false;
    }

    public void reset(){
        this.startTime = System.currentTimeMillis();
    }

    public boolean isRunning(){
        return this.running;
    }


    //elaspsed time in milliseconds
    public long getElapsedTime() {
        long elapsed;
        if (running) {
            elapsed = (System.currentTimeMillis() - startTime);
        }
        else {
            elapsed = (stopTime - startTime);
        }
        return elapsed;
    }


    //elaspsed time in seconds
    public double getElapsedTimeSecs() {
        double elapsed;
        if (running) {
            elapsed = ((System.currentTimeMillis() - startTime) / 1000);
        }
        else {
            elapsed = ((stopTime - startTime) / 1000);
        }
        return elapsed;
    }
}