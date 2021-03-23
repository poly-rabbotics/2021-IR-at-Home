package frc.robot.utils;

public class KGains {
    public double kP, kI, kD, kF;
    public KGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}