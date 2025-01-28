package frc.robot.modules;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.PID;

public class OldSwerveModule {
    private final SparkMax driveMotor;
    private final TalonSRX turnMotor;

    private double invert;
    private double angle;

    private final PID pid;

    public void setState(SwerveModuleState state) {
        double err = errCalculator(state.angle.getDegrees() - getEncValue());
        driveMotor.set(state.speedMetersPerSecond / Constants.RobotConstants.MaxSpeed * invert);
        turnMotor.set(TalonSRXControlMode.PercentOutput, pid.calculate(err / 90.0) );
    }

    private double getEncValue() {
        angle = ((int)turnMotor.getSelectedSensorPosition() & 0x03ff) * 0.3515625;
        if(angle < -180) angle += 360;
        if(angle > 180) angle -= 360;
        return angle;
    }

    private double errCalculator(double err) {
		if(invert == -1){
			err -= 180;
			err = err < -180 ? err + 360 : err;
		}

		err = err > 180 ? err - 360 : err;
		err = err < -180 ? err + 360 : err;

		if(-90 <= err && err < 90){}
		else if(90 <= err && err < 180){
			err -= 180;
			invert *= -1.0;
		}
		else if(-180 <= err && err < -90){
			err += 180;
			invert *= -1.0;
		}
        return err;
    }

    public void logging(String name) {
        SmartDashboard.putNumber(name, angle);
    }

    public OldSwerveModule(int driveMotorId, int turnMotorId) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushed);
        turnMotor = new TalonSRX(turnMotorId);

        pid = new PID(1.0, 2*1e-3, 0);

        invert = 1.0;
    }
}
