package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double kP, kI, kD, currentPosition, targetPosition, currentTime, previousTime, currentError, previousError;
    private ElapsedTime timer;

    public PIDController (double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        timer.reset();
    }

    public void setP (double kP) {
        this.kP = kP;
    }

    public void setI (double kI) {
        this.kI = kI;
    }

    public void setD (double kD) {
        this.kD = kD;
    }

    public void setPID (double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }


    public double calculateOutput (double current, double target) {
        previousTime = currentTime;
        previousError = currentError;

        currentPosition = current;
        targetPosition = target;
        currentTime = timer.seconds();

        currentError = targetPosition - currentPosition;

        double output = 0;

        output += kP * currentError;
        output += kI * (currentError * (currentTime - previousTime));
        output += kD * ((currentError - previousError) / (currentTime - previousTime));

        return output;
    }
}
