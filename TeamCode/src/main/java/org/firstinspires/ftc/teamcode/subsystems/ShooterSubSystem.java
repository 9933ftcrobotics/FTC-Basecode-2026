package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubSystem {
    Telemetry telemetry;
    DcMotorEx shooterMotor;
    public ShooterSubSystem(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry=telemetry;
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

    }

    public void spinUp(double speed){
        shooterMotor.setVelocity(speed);
        telemetry.addData("shooterzoom", (shooterMotor.getVelocity()));

    }

    public void stop(){
        shooterMotor.setPower(0);

    }
}
