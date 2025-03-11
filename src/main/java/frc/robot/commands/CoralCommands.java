// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Wrist;

public class CoralCommands {
  public static Command intake(Wrist wrist) {
    return Commands.runOnce(
        () -> {
          wrist.intakeSpeed(0.25);
        },
        wrist);
  }

  public static Command outake(Wrist wrist) {
    return Commands.runOnce(
        () -> {
          wrist.intakeSpeed(-0.25);
        },
        wrist);
  }

  public static Command stop(Wrist wrist) {
    return Commands.runOnce(
        () -> {
          wrist.intakeSpeed(0);
        },
        wrist);
  }
}
