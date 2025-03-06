// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgeaIntake extends SubsystemBase {
  private final SparkMax leftIntake;
  private final SparkMax rightIntake;

  public AlgeaIntake() {
    leftIntake = new SparkMax(3, MotorType.kBrushless);
    rightIntake = new SparkMax(1, MotorType.kBrushless);
  }

  @Override
  public void periodic() {}

  public void intake(boolean moveIntake) {
    if (moveIntake) {
      leftIntake.set(1);
      rightIntake.set(1);
    } else {
      leftIntake.set(0);
      rightIntake.set(0);
    }
  }
}
