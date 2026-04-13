package frc.robot.subsystems.Swerve;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveState extends SubsystemBase {
    private final Swerve swerveState;
    private final Function<Swerve, Swerve> update;
    private final Function<Swerve, Void> display;

    public SwerveState(Swerve swerveState, Function<Swerve, Swerve> update, Function<Swerve, Void> display) {
        this.swerveState = swerveState;
        this.update = update;
        this.display = display;
    }


    @Override
    public void periodic() {
        update.apply(swerveState);
        display.apply(swerveState);
    }
}
