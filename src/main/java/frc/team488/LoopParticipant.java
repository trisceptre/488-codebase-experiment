package frc.team488;

/**
 * Something the robot loop reads at the start of a cycle and writes at the end. {@code Robot}
 * calls every participant's {@link #updateInputs()}, then runs the command scheduler, then calls
 * every {@link #applyOutputs()}.
 */
public interface LoopParticipant {
    void updateInputs();

    void applyOutputs();
}
