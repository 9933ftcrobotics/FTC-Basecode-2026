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
        if (tresStep == 0) {
            drive.seedPose(0, 0, 0);
            tresStep = 10;
        }
        else if (tresStep == 10) {
            drive.driveToPose(66, 7, -34);
            if (drive.isRobotAtTarget()) {
                tresStep = 20;
            }
        }
        else if (tresStep == 20) {
            shootTres();
            literallyAnything = shootTres();
            if (literallyAnything) {
                tresStep = 30;
            }
        }
        else if (tresStep == 30) {
            drive.driveToPose(80,23,0);
            if (drive.isRobotAtTarget()) {
            tresStep = 40;}
        }
        else if (tresStep == 40) {
            intake.start();
            sleep(1000);
            tresStep = 50;
        }
        else if (tresStep == 50) {
            drive.driveToPose(109,19,0);
            if (drive.isRobotAtTarget()) {
            tresStep = 60;}
        }
        else if (tresStep == 60) {
            drive.driveToPose(70, 6, -40);
            if (drive.isRobotAtTarget()) {
            tresStep = 70;}
        }
        else if (tresStep == 70) {
            shootTres();
            tresStep = 80;
        }

        }

    }




