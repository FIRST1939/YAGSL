package frc.robot.commands.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Constants;

public class TrackAprilTags extends Command {
    
    private final Swerve swerve;
    private final Limelight limelight;

    public TrackAprilTags (Swerve swerve, Limelight limelight) {

        this.swerve = swerve;
        this.limelight = limelight;
        this.addRequirements(this.limelight);
    }

    @Override
    public void execute () {

        if (!this.limelight.areValidMeasurements()) { return; }

        double distanceToTag = this.limelight.getLatestDistance().getNorm();
        double distanceToTag2 = distanceToTag * distanceToTag;

        Matrix<N3, N1> standardDeviations = MatBuilder.fill(
            Nat.N3(), Nat.N1(), 
            Constants.SwerveConstants.LIMELIGHT_DEFAULT_DEVIATIONS.get(0, 0) * distanceToTag2,
            Constants.SwerveConstants.LIMELIGHT_DEFAULT_DEVIATIONS.get(1, 0) * distanceToTag2,
            Constants.SwerveConstants.LIMELIGHT_DEFAULT_DEVIATIONS.get(2, 0) * distanceToTag2
        );

        this.swerve.addVisionMeasurement(
            this.limelight.getLatestPose(), 
            Timer.getFPGATimestamp() - (this.limelight.getLatestDelay() / 1000.0),
            standardDeviations
        );
    }

    @Override
    public boolean isFinished () { return false; }
}
