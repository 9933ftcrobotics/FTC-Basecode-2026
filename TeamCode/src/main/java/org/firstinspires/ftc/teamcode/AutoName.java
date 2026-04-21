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

        switch (step) {


            case 0:
                drive.seedPose(0, 0, 0);
                step = 10;
                break;
            case 10:
                drive.driveToPose(70, 6, -40);
                if (drive.isRobotAtTarget()) {
                    step = 30;
                }
                break;
            case 30:

                literallyAnything = shootTres();
                if (literallyAnything) {
                    step = 40;
                }
                break;
            case 40:
                drive.driveToPose(80,23,0);
                if (drive.isRobotAtTarget()) {
                    step = 50;
                }
                break;
            case 50:
                //sleep(4000);
                intake.start();
                sleep(1000);
                step = 60;
                break;
            case 60:
                drive.driveToPose(109,19,0);
                if (drive.isRobotAtTarget()) {
                    step = 80;
                }
                break;
            case 70:
                drive.driveToPose(70, 6, -40);
                if (drive.isRobotAtTarget()) {
                    step = 70;
                }
                break;
            case 80:
                shootTres();
                break;
            default:
                break;
        }

    }

}


