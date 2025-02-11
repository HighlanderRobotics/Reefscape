// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.google.common.base.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot.ReefTarget;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  public static final int LED_LENGTH = 16;
  public static final int LED_ID = 3;
  private final LEDIO io;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();
  private double rainbowStart = 0;
  private double dashStart = 0;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LED", inputs);
  }

  private void setIndex(int i, Color color) {
    io.set(i, color);
  }

  private void setSolid(Color color) {
    io.solid(color);
  }

  public Command setSolidCmd(Color color) {
    return this.run(() -> setSolid(color));
  }

  public Command setBlinkingCmd(Color onColor, Color offColor, double frequency) {
    return Commands.repeatingSequence(
        setSolidCmd(onColor).withTimeout(1.0 / frequency),
        setSolidCmd(offColor).withTimeout(1.0 / frequency));
  }

  public Command setSolidCmd(Supplier<Color> color) {
    return this.run(() -> setSolid(color.get()));
  }

  public Command setBlinkingCmd(
      Supplier<Color> onColor, Supplier<Color> offColor, double frequency) {
    return Commands.repeatingSequence(
        setSolidCmd(onColor).withTimeout(1.0 / frequency),
        setSolidCmd(offColor).withTimeout(1.0 / frequency));
  }

  /** Sets the first portion of the leds to a color, and the rest off */
  public Command setProgressCmd(Color color, DoubleSupplier progress) {
    return this.run(
        () -> {
          for (int i = 0; i < LED_LENGTH; i++) {
            setIndex(i, i < progress.getAsDouble() * LED_LENGTH ? color : Color.kBlack);
          }
        });
  }

  public Command setRainbowCmd() {
    return this.run(
        () -> {
          for (int i = 0; i < LED_LENGTH; i++) {
            setIndex(i, Color.fromHSV((int) rainbowStart % 180 + i, 255, 255));
          }
          rainbowStart += 6;
        });
  }

  public Command setRunAlongCmd(
      Supplier<Color> colorDash, Supplier<Color> colorBg, int dashLength, double frequency) {
    return this.run(
        () -> {
          setSolid(colorBg.get());
          for (int i = (int) dashStart; i < dashStart + dashLength; i++) {
            setIndex(i % LED_LENGTH, colorDash.get());
          }

          dashStart += LED_LENGTH * frequency * 0.020;
          dashStart %= LED_LENGTH;
        });
  }

  public Command defaultStateDisplayCmd(
      BooleanSupplier enabled, BooleanSupplier inRange, Supplier<ReefTarget> target) {
    return Commands.either(
            Commands.select(
                Map.of(
                    ReefTarget.L1,
                    this.setBlinkingCmd(
                            () -> new Color("#ffff00"), // yellow
                            () ->
                                inRange.getAsBoolean() ? new Color("#0000ff") : new Color(), // blue
                            10.0)
                        .until(
                            () ->
                                target.get() != ReefTarget.L1
                                    || !enabled.getAsBoolean()), // stop when target changes
                    ReefTarget.L2,
                    this.setBlinkingCmd(new Color("#ff7777"), new Color(), 10.0) // pink
                        .until(() -> target.get() != ReefTarget.L2 || !enabled.getAsBoolean()),
                    ReefTarget.L3,
                    this.setBlinkingCmd(new Color("#0000ff"), new Color(), 10.0) // blue
                        .until(() -> target.get() != ReefTarget.L3 || !enabled.getAsBoolean()),
                    ReefTarget.L4,
                    this.setBlinkingCmd(new Color("#ff0000"), new Color(), 10.0) // red
                        .until(() -> target.get() != ReefTarget.L4 || !enabled.getAsBoolean())),
                target),
            this.setRunAlongCmd(
                    // Set color to be purple with a moving dash corresponding to alliance color
                    () -> {
                      if (DriverStation.getAlliance().isEmpty()) {
                        return new Color("#b59aff"); // Purple
                      } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                        return new Color("#ff0000"); // Red
                      } else { // Blue
                        return new Color("#0000ff"); // Blue
                      }
                    },
                    () -> new Color("#350868"), // Dark purple
                    10,
                    1.0)
                .until(enabled),
            enabled)
        .ignoringDisable(true)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .repeatedly();
  }
}
