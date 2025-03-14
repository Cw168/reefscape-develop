// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Funnel extends SubsystemBase {
  // Hardware
  private final TalonFX talon;
  // private final DutyCycleEncoder wristEncoder;
  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0);

  public static final double reduction =
      1; // wrist gearbox gear ration 60.0 * 60.0 * 30.0 / (10.0 * 18.0 * 12.0)
  // horizontal
  public static final double minAngle = 20;
  public static final double maxAngle = 80;

  double targetDegrees = minAngle;

  @AutoLog
  public static class FunnelIOInputs {
    public double targetAngle = 0.0;
    public double currentAngle = 0.0;
  }

  public final FunnelIOInputsAutoLogged pivotInputs = new FunnelIOInputsAutoLogged();

  public Funnel() {
    talon = new TalonFX(20, TunerConstants.kCANBus);

    // Configure motor
    TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armTalonConfig.Feedback.SensorToMechanismRatio = reduction;
    armTalonConfig.Feedback.RotorToSensorRatio = 1;

    armTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    // Move the arm
    armTalonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot0.kG = 0.35; // 0.35; // to hold the arm weight
    armTalonConfig.Slot0.kP = 10; // 60; // 100; // adjust PID
    armTalonConfig.Slot0.kI = 0;
    armTalonConfig.Slot0.kD = 0.01;
    armTalonConfig.Slot0.kS = 0;
    armTalonConfig.Slot0.kV = 5; // 8.3; // move velocity
    armTalonConfig.Slot0.kA = 0.15; // 0.2; // move accerleration

    armTalonConfig.MotionMagic.MotionMagicCruiseVelocity = 8; // 1.0; // 0.5;
    armTalonConfig.MotionMagic.MotionMagicAcceleration = 2; // 2; // 1.0;
    armTalonConfig.MotionMagic.MotionMagicJerk = 20; // 10; // 10;

    pMmPos.Slot = 0;
    pMmPos.EnableFOC = true;

    // Set up armTalonConfig
    talon.getConfigurator().apply(armTalonConfig, 0.25);

    // ParentDevice.optimizeBusUtilizationForAll(talon, wristEncoder);
  }

  @Override
  public void periodic() {
    pivotInputs.targetAngle = targetDegrees;
    pivotInputs.currentAngle = getAngle();
    Logger.processInputs("Funnel", pivotInputs);
    talon.setControl(pMmPos.withPosition(Units.degreesToRotations(targetDegrees)));
    if (DriverStation.isDisabled()) {
      talon.setControl(new NeutralOut());
    }
  }

  public double getAngle() {
    return talon.getPosition().getValueAsDouble() * 360;
  }

  public void setWristAngle(double setPointAngle) {
    targetDegrees = MathUtil.clamp(setPointAngle, minAngle, maxAngle);
  }
}
