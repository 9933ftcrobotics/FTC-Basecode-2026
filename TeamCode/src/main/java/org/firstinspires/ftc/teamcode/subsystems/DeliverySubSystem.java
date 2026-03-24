package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DeliverySubSystem {
    Telemetry telemetry;
    DcMotorEx beltMotor;
    CRServo transferWheel;

    public DeliverySubSystem(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;

        beltMotor = hardwareMap.get(DcMotorEx.class, "belt");
        transferWheel = hardwareMap.get(CRServo.class,"transferwheel");
        beltMotor.setDirection(DcMotorEx.Direction.REVERSE);
        transferWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setDeliverySpeed ( double speed) {

        beltMotor.setPower(speed);
        transferWheel.setPower(speed);


    }
}














