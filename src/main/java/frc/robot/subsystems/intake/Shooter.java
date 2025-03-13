// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.ShooterConstants;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;

public class Shooter extends SubsystemBase {
  private final TalonFX transferMoter;

  private final TalonFX leftShooter;
  private final TalonFX rightShooter;

  @AutoLog
  public static class ShooterIOInputs {
    public double leftRotationSpeed;
    public double rightRotationSpeed;
  }

  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter() {
    transferMoter = new TalonFX(17, TunerConstants.kCANBus);

    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    transferMoter.getConfigurator().apply(shooterConfig);

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
    transferMoter.set(speed * 0.3);
  }

  public void transfer(boolean moveTransfer) {
    if (moveTransfer) {
      transferMoter.set(0.25);
    } else {
      transferMoter.set(0);
      transferMoter.stopMotor();
    }
  }
}
