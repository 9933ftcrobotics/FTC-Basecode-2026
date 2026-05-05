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
        if(gamepad1.a){
            intake.start();
        }
        else{
            intake.stop();
        }
        if(gamepad1.right_trigger>.25) {
            shooter.spinUp(2000);
        }
        else{
            shooter.stop();
            if (gamepad1.a) {
                delivery.beltMove(100);
                delivery.ShooterDelivtobelt(20);
            }
            else delivery.stop();
        }
        if(gamepad1.start){
            drive.seedPose(0,0,0);
        }
        if (gamepad1.b) {
            delivery.ShooterDelivArm(1);
        } else {
            delivery.ShooterDelivArm(0);
        }
    }
}
