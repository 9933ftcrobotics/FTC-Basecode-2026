package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class BasicTeleOp extends TurtleOpMode {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
        drive.fieldCentricDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if(gamepad1.left_bumper){
            intake.start();
        }
        else{
            intake.stop();
        }
        if(gamepad1.start){
            drive.seedPose(0,0,0);
        }

        if (gamepad1.y){
            shooter.setShooterSpeed(1);
        }
        else{
            shooter.setShooterSpeed(0);
        }
        if (gamepad1.left_bumper){
            delivery.setDeliverySpeed(1);
        }
        else {
            delivery.setDeliverySpeed(0);
    }
        if (gamepad1.right_bumper){
            shooter.setLoading(0.25);
        }
        else {
            shooter.setLoading(0.075);
        }

    }

    @Override
    public void stop() {super.stop();}
}
