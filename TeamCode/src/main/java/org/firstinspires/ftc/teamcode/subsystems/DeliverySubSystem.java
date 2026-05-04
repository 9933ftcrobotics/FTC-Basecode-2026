package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeliverySubSystem {
    Telemetry telemetry;
    DcMotorEx deliveryMotor;


    public DeliverySubSystem(Telemetry telemetry, HardwareMap hardwareMap) {

        this.telemetry=telemetry;
            deliveryMotor = hardwareMap.get(DcMotorEx.class, "delivery");

}
        public void beltMove(double speed) {
            deliveryMotor.setVelocity(speed);

        }


}

