package org.firstinspires.ftc.teamcode.turtleUtils;

public class PIDController {

    private double p = 0, i = 0, d = 0;
    private double error = 0, prevError = 0, totalError = 0;

    public PIDController (double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public PIDController (double p) {
        this(p, 0, 0);
    }

    //////////////////////////// Getters /////////////////////

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    //////////////////////////// Setters /////////////////////

    public void setP(double p) {
        this.p = p;
    }

    public void setI(double i) {
        this.i = i;
    }

    public void setD(double d) {
        this.d = d;
    }

    ///////////////////////// Calculation Method ///////////////

    //Pulled from WPILIB, simplified to remove some integral madness
    public double calculate (double setpoint, double measurement) {
        prevError = error;

        error = setpoint - measurement;

        double errorDerivative = (error - prevError) / 0.02;

        if (i != 0) {
            totalError =
                    Math.max(-1.0 / i, Math.min(totalError + error * 0.02, 1.0 / i));
        }

        return p * error + i * totalError + d * errorDerivative;
    }


}
