// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ShooterConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public class Shooter extends SubsystemBase {
  private final SparkMax leftTransfer;
  private final SparkMax rightTransfer;

  private final SparkMax leftIntake;
  private final SparkMax rightIntake;

  private final TalonFX leftShooter;
  private final TalonFX rightShooter;

  @AutoLog
  public static class ShooterIOInputs {
    public double leftRotationSpeed;
    public double rightRotationSpeed;
  }

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {
    leftTransfer = new SparkMax(4, MotorType.kBrushless);
    rightTransfer = new SparkMax(2, MotorType.kBrushless);

    leftIntake = new SparkMax(5, MotorType.kBrushless);
    rightIntake = new SparkMax(3, MotorType.kBrushless);

    rightShooter = new TalonFX(13, TunerConstants.kCANBus);
    leftShooter = new TalonFX(14, TunerConstants.kCANBus);
  }

  @Override
  public void periodic() {
    inputs.leftRotationSpeed = leftShooter.getVelocity().getValueAsDouble();
    inputs.rightRotationSpeed = rightShooter.getVelocity().getValueAsDouble();
  }

  public void shoot(boolean moveShooter) {
    if (moveShooter) {
      rightShooter.setVoltage(-ShooterConstants.shooterVoltage);
      leftShooter.setVoltage(ShooterConstants.shooterVoltage);
    } else {

      rightShooter.setVoltage(0);
      leftShooter.setVoltage(0);
    }
  }

  public void transferSpeed(double speed) {
    rightTransfer.set(-speed * 0.3);
    leftTransfer.set(speed * 0.3);
  }

  public void transfer(boolean moveTransfer) {
    if (moveTransfer) {
      rightTransfer.set(-0.25);
      leftTransfer.set(0.25);
    } else {
      leftTransfer.set(0);
      rightTransfer.set(0);
      leftTransfer.stopMotor();
      rightTransfer.stopMotor();
    }
  }

  public void intake(double moveTransfer) {
    if (moveTransfer == 1) {
      rightIntake.set(-0.5);
      leftIntake.set(0.5);
    } else if (moveTransfer == -1) {
      rightIntake.set(0.5);
      leftIntake.set(-0.5);
    } else {
      leftIntake.set(0);
      rightIntake.set(0);
      leftIntake.stopMotor();
      rightIntake.stopMotor();
    }
  }

  public void intakeMove(double speed) {
    rightIntake.set(-1);
    leftIntake.set(1);
  }
}
