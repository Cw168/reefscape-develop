// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class Elevator extends SubsystemBase {
  public static final double ELEVATOR_GEAR_REDUCTION = 5.0;
  public static final double ELEVATOR_SPROCKET_PERIMETER =
      Units.inchesToMeters(2.472433418375167278670100342641) * 100; // inches
  private static final double POS_SWITCH_THRESHOLD = 2;
  public static final double minHeight = 0;
  public static final double maxHeight = 60;

  double targetHeight = SuperStructureState.SOURCE_HEIGHT;

  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0);

  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motorConnected = true;
    public double motorPosition = 0;
    public double targetHeight = 0;
    public double elevatorHeight = 0;
  }

  private boolean manuelMoving = false;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator() {
    talon = new TalonFX(12, TunerConstants.kCANBus);
    followerTalon = new TalonFX(18, TunerConstants.kCANBus);
    followerTalon.setControl(new Follower(talon.getDeviceID(), true));
    talon.setPosition(0);
    // Configure motor
    TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armTalonConfig.Feedback.RotorToSensorRatio = 1; // ELEVATOR_GEAR_REDUCTION
    armTalonConfig.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_REDUCTION;

    armTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    // Move the arm
    armTalonConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    armTalonConfig.Slot0.kG = 0.3; // 0.35; // 0.35; // to hold the arm weight
    armTalonConfig.Slot0.kP = 60; // 40; // 60; // 100; // adjust PID
    armTalonConfig.Slot0.kI = 0;
    armTalonConfig.Slot0.kD = 0;
    armTalonConfig.Slot0.kS = 0;

    armTalonConfig.MotionMagic.MotionMagicCruiseVelocity = 70; // 1.0; // 0.5;
    armTalonConfig.MotionMagic.MotionMagicAcceleration = 10; // 2; // 1.0;
    armTalonConfig.MotionMagic.MotionMagicJerk = 0; // 10; // 10;

    pMmPos.Slot = 0;
    pMmPos.EnableFOC = true;
    // Set up armTalonConfig
    tryUntilOk(5, () -> talon.getConfigurator().apply(armTalonConfig, 0.25));

    // ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  public void manualMove(double velocity) {
    SmartDashboard.putString("Elevator", "Manual");
    if (velocity != 0) manuelMoving = true;
    else manuelMoving = false;
    talon.set(-velocity);
  }

  public void setVoltage(double voltage) {
    // Set the power to the main motor
    talon.setControl(new VoltageOut(voltage));
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    inputs.motorConnected = talon.isConnected();
    inputs.motorPosition = talon.getPosition().getValueAsDouble();
    inputs.targetHeight = targetHeight;
    inputs.elevatorHeight = inputs.motorPosition * ELEVATOR_SPROCKET_PERIMETER;

    Logger.processInputs("Elevator", inputs);

    // if (Math.abs(targetHeight - inputs.elevatorHeight) < POS_SWITCH_THRESHOLD) {
    //   if (targetHeight == SuperStructureState.SOURCE_HEIGHT
    //       && Math.abs(targetHeight - inputs.elevatorHeight) < 0.5) { // degree
    //     talon.setControl(new NeutralOut());
    //   } else {
    //     talon.setControl(
    //         pPos.withPosition(
    //             targetHeight / ELEVATOR_SPROCKET_PERIMETER * ELEVATOR_GEAR_REDUCTION));
    //   }
    // } else {
    //
    if (!manuelMoving)
      talon.setControl(pMmPos.withPosition(targetHeight / ELEVATOR_SPROCKET_PERIMETER));
    // }

    if (DriverStation.isDisabled()) {
      talon.setControl(new NeutralOut());
    }

    // setElevatorHeight(getElevatorHeight());
  }

  public void resetPosition() {
    // Reset the encoder to the specified position
    talon.setPosition(0);
  }

  public void stop() {
    targetHeight = inputs.elevatorHeight;
  }

  public void setElevatorHeight(double setPointHeight) {

    targetHeight = MathUtil.clamp(setPointHeight, minHeight, maxHeight);
  }

  public double getElevatorHeight() {
    return inputs.elevatorHeight;
  }

  public BooleanSupplier isDone() {
    boolean flag = Math.abs(targetHeight - inputs.elevatorHeight) < 2;
    return () -> flag;
  }
}
