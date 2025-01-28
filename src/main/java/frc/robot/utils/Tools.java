package frc.robot.utils;

public class Tools {
	//vector (x, y) convert to degrees 
	public static double toDegrees(final double x, final double y) {
		if(x == 0 && y == 0) return 0;

		double angle = 90 - Math.atan2(y, x) * 57.2957805;//... / 3.1415926 * 180.0

    return angle < 0 ? angle + 360 : angle;
	}

	//degrees convert to vector (x, y) 
	public static double[] toVector(final double radius, double angle) {
    if(angle < 0) angle += 360;
    else if(angle > 360) angle -= 360;

		final double rad = angle * 0.0174533;//angle * 3.1415926 / 180.0

		return new double[]{radius * Math.sin(rad), radius * Math.cos(rad)};
	}

	//bounding 
	public static double bounding(double pos, final double min, final double max) {
		pos = pos < min ? min : pos;
		pos = pos > max ? max : pos;

		return pos;
	}

	//bounding 1 ~ -1
	public static double bounding(double pos) {
		pos = pos < -1 ? -1 : pos;
		pos = pos > 1 ? 1 : pos;

		return pos;
	}

	//check deadband
	public static double deadband(double value, final double deadbandValue) {
		return -deadbandValue < value && value < deadbandValue ? 0 : value;
	}

}