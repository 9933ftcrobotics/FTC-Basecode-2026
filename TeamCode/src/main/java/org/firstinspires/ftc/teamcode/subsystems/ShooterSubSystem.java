package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubSystem {
    Telemetry telemetry;
    DcMotorEx shooterMotor;
    Servo loading;

    public ShooterSubSystem(Telemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        loading = hardwareMap.get(Servo.class, "loading");
    }


    public void setShooterSpeed (double speed) {

      shooterMotor.setPower(speed);
    }
    public void setLoading (double position) {

        loading.setPosition(position);
    }

}
