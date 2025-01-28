package frc.robot.utils;

public class PID {
	private double kp;
	private double ki;
	private double kd;

	private double Integral;
	private double errorArray[];
	private double deadband;

	private int thisError;
	private int lastError;

	public PID(double _kp, double _ki, double _kd) {
		kp = _kp;
		ki = _ki;
		kd = _kd;

		deadband = 0;
		resetIntergral();
	}

	public double calculate(final double error) {
		if(-deadband < error && error < deadband){
			resetIntergral();
			return 0;
		}
		thisError++;
		if(thisError == 50) thisError = 0;

		if(thisError == 0) lastError = 49;
		else lastError = thisError - 1;

		//P
		final double P = error * kp;

		//I
		Integral += (error - errorArray[thisError]) * ki;
		final double I = Integral;

		//D
		final double D = (error - errorArray[lastError]) * kd;
		errorArray[thisError] = error;

		return P + I + D;
	}

	public void setDeadband(final double deadband){
		this.deadband = deadband;
	}

	public void resetIntergral() {
		errorArray = new double[50];// 1s
		Integral = 0;

		thisError = 0;
		lastError = 49;
	}
	
}