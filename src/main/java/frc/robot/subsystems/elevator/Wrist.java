// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructureState;
import frc.robot.generated.TunerConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import frc.robot.RobotContainer;
import frc.robot.Robot;

public class Wrist extends SubsystemBase {
  private RobotContainer robotContainer;

  // Hardware
  private final TalonFX talon;
  private final TalonFX wristIntake;
  // private final DutyCycleEncoder wristEncoder;
  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0);

  public static final double reduction =
      75; // wrist gearbox gear ration 60.0 * 60.0 * 30.0 / (10.0 * 18.0 * 12.0)
  // horizontal
  public static final double minAngle = -175;
  public static final double maxAngle = -80;

  double targetDegrees = SuperStructureState.SOURCE_ANGLE;

  @AutoLog
  public static class WristIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = false;
    public double targetAngle = 0.0;
    public double currentAngle = 0.0;
  }

  public final WristIOInputsAutoLogged pivotInputs = new WristIOInputsAutoLogged();

  public Wrist() {
    talon = new TalonFX(16, TunerConstants.kCANBus);
    wristIntake = new TalonFX(17, TunerConstants.kCANBus);
    // DigitalInput input = new DigitalInput(9);
    // wristEncoder = new DutyCycleEncoder(input);

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
    armTalonConfig.Slot0.kP = 40; // 60; // 100; // adjust PID
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
    wristIntake.getConfigurator().apply(armTalonConfig, 0.25);

    // ParentDevice.optimizeBusUtilizationForAll(talon, wristEncoder);
  }

  public boolean isSafe() {
    return pivotInputs.currentAngle < 80;
  }

  public void periodic() {
    pivotInputs.encoderConnected = false;
    pivotInputs.motorConnected = talon.isConnected();
    pivotInputs.targetAngle = targetDegrees;
    pivotInputs.currentAngle = getAngle();
    Logger.processInputs("Wrist", pivotInputs);
    talon.setControl(pMmPos.withPosition(Units.degreesToRotations(targetDegrees)));
    if (DriverStation.isDisabled()) {
      talon.setControl(new NeutralOut());
    }
    SmartDashboard.putNumber("TarAngle", targetDegrees);
    SmartDashboard.putNumber("CurAngle", pivotInputs.currentAngle);
  }

  public void setVoltage(double voltage) {
    // Set the power to the main motor
    talon.setControl(new VoltageOut(voltage));
  }

  public void moveWrist(double moveWrist) {
    if (moveWrist == 0) {
      talon.set(0);
      talon.stopMotor();
    } else {
      if (pivotInputs.currentAngle <= maxAngle + 10 && pivotInputs.currentAngle >= minAngle - 10) {
        talon.set(moveWrist);
      } else if (pivotInputs.currentAngle >= maxAngle
        && !robotContainer.m_controller.getXButton()
        && !robotContainer.m_controller.getYButton()) {
        talon.set(0.1);
      } else if (pivotInputs.currentAngle <= minAngle
          && !robotContainer.m_controller.getXButton()
          && !robotContainer.m_controller.getYButton()) {
        talon.set(-0.1);
      }
    }
  }

  public double getAngle() {
    return talon.getPosition().getValueAsDouble() * 360;
  }

  public void moveIntake(double moveIntake) {
    if (moveIntake == 1) {
      wristIntake.set(1);
    } else if (moveIntake == -1) {
      wristIntake.set(-1);
    } else {
      wristIntake.set(0);
      wristIntake.stopMotor();
    }
  }

  public void setWristAngle(double setPointAngle) {
    targetDegrees = MathUtil.clamp(setPointAngle, minAngle, maxAngle);
  }

  public BooleanSupplier isDone() {
    boolean flag = Math.abs(targetDegrees - pivotInputs.currentAngle) < 3;
    return () -> flag;
  }
}
