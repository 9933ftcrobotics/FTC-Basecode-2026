package org.firstinspires.ftc.teamcode;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoName extends TurtleOpMode {

    int step = 0;
    boolean literallyAnything;
    @Override
    public void init() {

        super.init();
        //drive.seedPose(0,0,0);  <- Null Pointer Exception?
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addLine("Step: " + step);
        if (step == 0) {
            drive.seedPose(0, 0, 0);
            step = 10;
        }
        else if (step == 10) {
            drive.driveToPose(66, 7, -34);
            if (drive.isRobotAtTarget()) {
                step = 20;
            }
        }
        else if (step == 20) {
            shootTres();
            literallyAnything = shootTres();
            if (literallyAnything) {
                step = 30;
            }
        }
        else if (step == 30) {
            drive.driveToPose(80,23,0);
            if (drive.isRobotAtTarget()) {
                timer.reset();
                step = 40;
            }
        }
        else if (step == 40) {
            intake.start();
            if (timer.seconds() >= 1) {
                step = 50;
            }
        }
        else if (step == 50) {
            drive.driveToPose(109,19,0);
            if (drive.isRobotAtTarget()) {
                step = 60;}
        }
        else if (step == 60) {
            drive.driveToPose(70, 6, -40);
            if (drive.isRobotAtTarget()) {
                step = 70;}
        }
        else if (step == 70) {
            shootTres();
            step = 80;
        }

        }

    }




