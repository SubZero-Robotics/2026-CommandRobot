package frc.robot.utils;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.NeedsNameBuilderStage;

// Commands v3 equivalents of the Commands v2 InstantCommand, RunCommand, and
// ConditionalCommand, so v2 command factories port over with the same requirements and
// behavior.
public final class CommandUtils {
    private CommandUtils() {
    }

    // Equivalent of the v2 InstantCommand: runs the body once and finishes.
    public static NeedsNameBuilderStage runOnce(Runnable body, Mechanism... requirements) {
        return build(coroutine -> body.run(), requirements);
    }

    // Equivalent of the v2 RunCommand: runs the body every scheduler loop until it is canceled.
    public static NeedsNameBuilderStage runRepeatedly(Runnable body, Mechanism... requirements) {
        return build(coroutine -> {
            while (true) {
                body.run();
                coroutine.yield();
            }
        }, requirements);
    }

    // Equivalent of the v2 ConditionalCommand: checks the condition when it starts, then runs
    // onTrue or onFalse. Like v2, it requires everything either branch requires.
    public static NeedsNameBuilderStage conditional(Command onTrue, Command onFalse, BooleanSupplier condition) {
        Set<Mechanism> requirements = new HashSet<>(onTrue.requirements());
        requirements.addAll(onFalse.requirements());

        return build(coroutine -> coroutine.await(condition.getAsBoolean() ? onTrue : onFalse),
                requirements.toArray(Mechanism[]::new));
    }

    // Name of the command running on a mechanism, or "null" when it has none, like logging
    // the v2 Subsystem.getCurrentCommand().
    public static String currentCommandName(Mechanism mechanism) {
        List<Command> commands = mechanism.getRunningCommands();
        return commands.isEmpty() ? "null" : commands.getFirst().name();
    }

    private static NeedsNameBuilderStage build(Consumer<Coroutine> body, Mechanism... requirements) {
        if (requirements.length == 0) {
            return Command.noRequirements(body);
        }

        return Command.requiring(List.of(requirements)).executing(body);
    }
}
