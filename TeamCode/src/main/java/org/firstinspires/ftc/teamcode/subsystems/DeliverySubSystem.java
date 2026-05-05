package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeliverySubSystem {
    Telemetry telemetry;
    DcMotorEx deliveryMotor;
    private Servo shooterdeliveryarm;

    private CRServo shooterdeliverytobelt;
    public DeliverySubSystem(Telemetry telemetry, HardwareMap hardwareMap) {

        this.telemetry=telemetry;
        deliveryMotor = hardwareMap.get(DcMotorEx.class, "delivery");
        shooterdeliveryarm = hardwareMap.get(Servo.class, "shooter delivery arm");
        shooterdeliverytobelt = hardwareMap.get(CRServo.class, "shooter delivery to belt");
        deliveryMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterdeliverytobelt.setDirection(CRServo.Direction.REVERSE);

    }
        public void ShooterDelivArm(double speed){
            shooterdeliveryarm.setPosition(speed);
        }
    public void ShooterDelivtobelt(double speed){
        shooterdeliverytobelt.setPower(speed);
    }

        public void beltMove(double speed) {
            deliveryMotor.setVelocity(speed);

        }


    public void stop() {
        deliveryMotor.setPower(0);
        shooterdeliverytobelt.setPower(0);
    }
}

