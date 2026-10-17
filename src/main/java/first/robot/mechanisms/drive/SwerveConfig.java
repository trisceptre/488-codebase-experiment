package first.robot.mechanisms.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import java.util.List;

public record SwerveConfig(
        SwerveDrivetrainConstants drivetrain,
        List<SwerveModuleConstants<?, ?, ?>> modules,
        List<String> moduleLabels,
        double maxLinearSpeedMetersPerSec,
        double maxAngularSpeedRadPerSec) {

    public SwerveConfig {
        modules = List.copyOf(modules);
        moduleLabels = List.copyOf(moduleLabels);
        if (modules.size() != moduleLabels.size()) {
            throw new IllegalArgumentException("one label per module required");
        }
    }

    public static SwerveConfig create(
            SwerveDrivetrainConstants drivetrain,
            SwerveModuleConstants<?, ?, ?> frontLeft,
            SwerveModuleConstants<?, ?, ?> frontRight,
            SwerveModuleConstants<?, ?, ?> backLeft,
            SwerveModuleConstants<?, ?, ?> backRight,
            double maxLinearSpeedMetersPerSec) {
        List<SwerveModuleConstants<?, ?, ?>> modules = List.of(frontLeft, frontRight, backLeft, backRight);
        double driveBaseRadius = driveBaseRadiusMeters(modules);
        return new SwerveConfig(
                drivetrain,
                modules,
                List.of("FrontLeft", "FrontRight", "BackLeft", "BackRight"),
                maxLinearSpeedMetersPerSec,
                maxLinearSpeedMetersPerSec / driveBaseRadius);
    }

    public int moduleCount() {
        return modules.size();
    }

    public double driveBaseRadiusMeters() {
        return driveBaseRadiusMeters(modules);
    }

    public double wheelRadiusMeters() {
        return modules.get(0).WheelRadius;
    }

    public SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(modules.stream()
                .map(m -> new Translation2d(m.LocationX, m.LocationY))
                .toArray(Translation2d[]::new));
    }

    private static double driveBaseRadiusMeters(List<SwerveModuleConstants<?, ?, ?>> modules) {
        return modules.stream()
                .mapToDouble(m -> Math.hypot(m.LocationX, m.LocationY))
                .max()
                .orElseThrow();
    }
}
