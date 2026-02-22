package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubSystem {
    Telemetry telemetry;
    DcMotorEx intakeMotor;
    double defaultPower = 1;

    public IntakeSubSystem(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }


    public void start() {
        intakeMotor.setPower(defaultPower);
    }
    public void stop() {
        intakeMotor.setPower(0);
    }
    public void reverse() {
        intakeMotor.setPower(-defaultPower);
    }
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

}
