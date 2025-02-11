// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.google.common.base.Supplier;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
