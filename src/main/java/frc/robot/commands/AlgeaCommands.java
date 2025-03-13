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
import frc.robot.subsystems.intake.Shooter;

public class AlgeaCommands {
  private AlgeaCommands() {}

  public static Command Transfer(Shooter shooter, double transferSpd) {
    return Commands.runOnce(
        () -> {
          shooter.transferSpeed(transferSpd);
        },
        shooter);
  }

  public static Command shoot(Shooter shooter, boolean isShooting) {
    return Commands.runOnce(
        () -> {
          shooter.shoot(isShooting);
        },
        shooter);
  }

  public static Command positionIntake(Wrist wrist) {
    return Commands.run(
        () -> {
          wrist.setWristAngle(-100);
        },
        wrist);
  }
}
