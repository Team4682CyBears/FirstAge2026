// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ButtonPressCommand.java
// Intent: Forms a manual command to print the button number.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayDeque;

public class ButtonPressCommand extends Command {
  private static final int maxPreviousButtonCount = 10;
  private static final double roundPrecision = 100.0;
  private static ArrayDeque<String> previousButtons = new ArrayDeque<String>(ButtonPressCommand.maxPreviousButtonCount);
  private static Timer timer = new Timer();
  private static boolean timerStarted = false;

  private String inputDevice = "";
  private String inputAction = "";
  private double initTime = 0.0;
  private double executeTime = 0.0;
  private double finalTime = 0.0;

  /**
   * The constructor
   */
  public ButtonPressCommand(
      String inputDeviceDescription,
      String inputActionDescription) {
    inputDevice = inputDeviceDescription;
    inputAction = inputActionDescription;
    if (timerStarted == false) {
      timerStarted = true;
      timer.reset();
      timer.start();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = timer.get();
    executeTime = 0.0;
    finalTime = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    executeTime = timer.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addStringProperty("PreviousButtonPress0", ButtonPressCommand::getButtonDescription0, null);
    builder.addStringProperty("PreviousButtonPress1", ButtonPressCommand::getButtonDescription1, null);
    builder.addStringProperty("PreviousButtonPress2", ButtonPressCommand::getButtonDescription2, null);
    builder.addStringProperty("PreviousButtonPress3", ButtonPressCommand::getButtonDescription3, null);
    builder.addStringProperty("PreviousButtonPress4", ButtonPressCommand::getButtonDescription4, null);
    builder.addStringProperty("PreviousButtonPress5", ButtonPressCommand::getButtonDescription5, null);
    builder.addStringProperty("PreviousButtonPress6", ButtonPressCommand::getButtonDescription6, null);
    builder.addStringProperty("PreviousButtonPress7", ButtonPressCommand::getButtonDescription7, null);
    builder.addStringProperty("PreviousButtonPress8", ButtonPressCommand::getButtonDescription8, null);
    builder.addStringProperty("PreviousButtonPress9", ButtonPressCommand::getButtonDescription9, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finalTime = timer.get();
    String buttonPressDescription = this.toString();
    DataLogManager.log(buttonPressDescription);
    previousButtons.addFirst(buttonPressDescription);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public String toString() {
    return inputDevice + ":" +
        inputAction + ":" +
        Math.round(initTime * ButtonPressCommand.roundPrecision) / ButtonPressCommand.roundPrecision + ":" +
        Math.round(executeTime * ButtonPressCommand.roundPrecision) / ButtonPressCommand.roundPrecision + ":" +
        Math.round(finalTime * ButtonPressCommand.roundPrecision) / ButtonPressCommand.roundPrecision;
  }

  private static String getButtonDescription0() {
    return ButtonPressCommand.getButtonDescription(0);
  }

  private static String getButtonDescription1() {
    return ButtonPressCommand.getButtonDescription(1);
  }

  private static String getButtonDescription2() {
    return ButtonPressCommand.getButtonDescription(2);
  }

  private static String getButtonDescription3() {
    return ButtonPressCommand.getButtonDescription(3);
  }

  private static String getButtonDescription4() {
    return ButtonPressCommand.getButtonDescription(4);
  }

  private static String getButtonDescription5() {
    return ButtonPressCommand.getButtonDescription(5);
  }

  private static String getButtonDescription6() {
    return ButtonPressCommand.getButtonDescription(6);
  }

  private static String getButtonDescription7() {
    return ButtonPressCommand.getButtonDescription(7);
  }

  private static String getButtonDescription8() {
    return ButtonPressCommand.getButtonDescription(8);
  }

  private static String getButtonDescription9() {
    return ButtonPressCommand.getButtonDescription(9);
  }

  private static String getButtonDescription(int index) {
    // keep the button descriptions pruned to the right size
    for (int jnx = previousButtons.size() - ButtonPressCommand.maxPreviousButtonCount; jnx > 0; --jnx) {
      previousButtons.removeLast();
    }

    String rtnVal = "";
    if (previousButtons.size() > index) {
      String[] values = previousButtons.toArray(new String[0]);
      rtnVal = values[index];
    }
    return rtnVal;
  }
}
