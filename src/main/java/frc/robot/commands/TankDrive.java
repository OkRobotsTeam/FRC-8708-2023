package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final Supplier<Double> m_leftSpeedSupplier;
    private final Supplier<Double> m_rightSpeedSupplier;
    private final Supplier<Boolean> m_fastModeSupplier;

    public TankDrive(
        Drivetrain drivetrain,
        Supplier<Double> LeftSpeedSupplier,
        Supplier<Double> RightSpeedSupplier,
        Supplier<Boolean> FastModeSupplier
    ) {
        m_drivetrain = drivetrain;
        m_leftSpeedSupplier = LeftSpeedSupplier;
        m_rightSpeedSupplier = RightSpeedSupplier;
        m_fastModeSupplier = FastModeSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drivetrain.tankDrive(
            m_leftSpeedSupplier.get(),
            m_rightSpeedSupplier.get(),
            m_fastModeSupplier.get()
        );
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
