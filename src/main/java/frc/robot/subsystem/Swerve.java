package frc.robot.subsystem;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.modules.OldSwerveModule;
import frc.robot.utils.Tools;

public class Swerve extends SubsystemBase {
    private final OldSwerveModule front_left;
    private final OldSwerveModule front_right;
    private final OldSwerveModule back_left;
    private final OldSwerveModule back_right;

    private final AHRS gyro;
    private final double initialAngle;

    private final SwerveDriveKinematics kinematics;

    public void drive(double x, double y, double turn, boolean fieldRelative) {
        double invert = 1.0;
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) invert = -1.0;
        x = Tools.deadband(x, 0.1) * invert;
        y = Tools.deadband(y, 0.1) * invert;
        turn = Tools.deadband(turn, 0.1);

        SwerveModuleState states[] = kinematics.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(x, y, turn, Rotation2d.fromDegrees(getGyroAngle())):
                new ChassisSpeeds(x, y, turn) 
        );
        setModuleStates(states);
    }

    private double getGyroAngle() {
        double angle = 360 -  (gyro.getAngle() - initialAngle) + ((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) ? 180.0 : 0);
        angle %= 360;
        if(angle > 180) angle -= 360;
        if(angle < -180) angle += 360;
        return angle;
    }

    private void setModuleStates(SwerveModuleState states[]) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.RobotConstants.MaxSpeed);
        front_left.setState(states[0]);
        front_right.setState(states[1]);
        back_left.setState(states[2]);
        back_right.setState(states[3]);
    }

    public Swerve() {
        front_left = new OldSwerveModule(5, 1);
        front_right = new OldSwerveModule(6, 2);
        back_left = new OldSwerveModule(7, 3);
        back_right = new OldSwerveModule(8, 4);

        gyro = new AHRS(NavXComType.kMXP_SPI);
        initialAngle = gyro.getAngle();

        kinematics = new SwerveDriveKinematics(
            new Translation2d[] {
                new Translation2d(0.3, 0.3),
                new Translation2d(0.3, -0.3),
                new Translation2d(-0.3, 0.3),
                new Translation2d(-0.3, -0.3),
            }
        );
    }

    @Override 
    public void periodic() {
    }
}
