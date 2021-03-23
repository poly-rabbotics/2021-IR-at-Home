package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDController {
    private KGains gains;
    private boolean relationshipIsPositive;
    private double accumulatedError;
    private double lastMeasurement;
    private boolean lastMeasurementIsMeaningful;
    private double setpoint;
    public PIDController(KGains gains, boolean relationshipIsPositive) {
        this.gains = gains;
        this.relationshipIsPositive = relationshipIsPositive;
        reset();
    }
    public void reset() {
        lastMeasurementIsMeaningful = false;
        accumulatedError = 0;
    }
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    public double getSetpoint() {
        return setpoint;
    }
    public double getAccumulatedError() {
        return accumulatedError;
    }
    private double getPTerm(double error) {
        if(!relationshipIsPositive) {
            return gains.kP * error;
        }
        return - gains.kP * error;
    }
    private double getITerm(double error) {
        SmartDashboard.putNumber("Error at getITerm", error);
        SmartDashboard.putNumber("accumulated error at getITerm", getAccumulatedError());

        SmartDashboard.putNumber("math", error*getAccumulatedError());
        
        if(error * accumulatedError < 0) {
            //If the error is opposite of the error that has been accumulated
            //(i.e., we have crossed the setpoint and gone to the other side)
            //then error is zeroed.
            accumulatedError = 0;
        }
        accumulatedError = error + accumulatedError;
        if(relationshipIsPositive) {
            return -gains.kI * getAccumulatedError();
        }
        System.out.println(getAccumulatedError());
        return getAccumulatedError() * gains.kI;
    }
    private double restrictToInterval (double input, double lowerBound, double upperBound){
        double output = Math.min(upperBound, Math.max(lowerBound, input));
        return output;
    }
    private double getDTerm(double measurement) {
        if(!lastMeasurementIsMeaningful) {
            lastMeasurement = measurement;
            lastMeasurementIsMeaningful = true;
            return 0;
        }
        double ret = -(measurement - lastMeasurement) * gains.kD;
        if(!relationshipIsPositive) {
            ret = -ret;
        }
        lastMeasurement = measurement;
        return ret;
    }
    public double calculate(double measurement) {
        double error = measurement - setpoint;
        return restrictToInterval(getPTerm(error) + getITerm(error) + getDTerm(measurement), -1, 1);
    }

}