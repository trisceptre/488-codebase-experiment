package frc.team488.mechanisms.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;

/**
 * Everything that describes one swerve drivetrain: CTRE constants (from Tuner X or hand-written),
 * module log labels in the same order as {@code modules}, and the speed limits drive commands use.
 */
public record SwerveConfig(
        SwerveDrivetrainConstants drivetrain,
        List<SwerveModuleConstants<?, ?, ?>> modules,
        List<String> moduleLabels,
        double maxSpeedMetersPerSec,
        double maxAngularRateRadPerSec) {

    public SwerveConfig {
        modules = List.copyOf(modules);
        moduleLabels = List.copyOf(moduleLabels);
        if (modules.size() != moduleLabels.size()) {
            throw new IllegalArgumentException("one label per module required");
        }
    }

    public int moduleCount() {
        return modules.size();
    }

    public SwerveDriveKinematics kinematics() {
        return new SwerveDriveKinematics(modules.stream()
                .map(m -> new Translation2d(m.LocationX, m.LocationY))
                .toArray(Translation2d[]::new));
    }
}
