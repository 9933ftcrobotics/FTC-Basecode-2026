package org.firstinspires.ftc.teamcode.subsystems;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubSystem {
    Telemetry telemetry;
    DcMotorEx shooterMotor;
    Servo loading;
    double step;

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
    public boolean shootTres () {

    //before running make sure the robot is positioned
    // and the delivery + intake are running
        switch (step) {

            case 20:
                setShooterSpeed(.9);
                sleep(3000);
                step = 30;
                break;
            case 30:
                setLoading(.25);
                sleep(1200);
                step = 40;
                break;
            case 40:
                setLoading(0.075);
                sleep(1200);
                step = 50;
                break;
            case 50:
                setLoading(.25);
                sleep(1200);
                step = 60;
                break;
            case 60:
                setLoading(0.075);
                sleep(1200);
                step = 70;
                break;
            case 70:
                setLoading(.25);
                sleep(1200);
                step = 80;
                break;
            case 80:
                setLoading(0.075);
                sleep(1200);
                return true;
                break;
        }

}
