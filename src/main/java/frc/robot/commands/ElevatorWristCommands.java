// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SuperStructureState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Wrist;

// use this during driving
public class ElevatorWristCommands {
  public static Command setElevatorWristStage(Elevator elevator, Wrist wrist, int level) {

    return Commands.runOnce(
        () -> {
          double elevatorHeight = 0, wristAngle = 0;
          switch (level) {
            case 0 -> {
              elevatorHeight = SuperStructureState.L0_HEIGHT;
              wristAngle = SuperStructureState.L0_ANGLE;
            }

            case 1 -> {
              elevatorHeight = SuperStructureState.L1_HEIGHT;
              wristAngle = SuperStructureState.L1_ANGLE;
            }
            case 2 -> {
              elevatorHeight = SuperStructureState.L2_HEIGHT;
              wristAngle = SuperStructureState.L2_ANGLE;
            }
            case 3 -> {
              elevatorHeight = SuperStructureState.L3_HEIGHT;
              wristAngle = SuperStructureState.L3_ANGLE;
            }
            case 4 -> {
              elevatorHeight = SuperStructureState.L4_HEIGHT;
              wristAngle = SuperStructureState.L4_ANGLE;
            }
            case 5 -> {
              elevatorHeight = SuperStructureState.TRANSFER_HEIGHT;
              wristAngle = SuperStructureState.TRANSFER_ANGLE;
            }
            case 6 -> {
              elevatorHeight = SuperStructureState.L1B_HEIGHT;
              wristAngle = SuperStructureState.L1B_ANGLE;
            }
            case 7 -> {
              elevatorHeight = SuperStructureState.L2B_HEIGHT;
              wristAngle = SuperStructureState.L2B_ANGLE;
            }
            case 8 -> {
              elevatorHeight = SuperStructureState.TRANSFER_HEIGHT;
              wristAngle = SuperStructureState.TRANSFER_ANGLE;
            }
          }
          elevator.setElevatorHeight(elevatorHeight);
          wrist.setWristAngle(wristAngle);
        },
        elevator,
        wrist);
  }

  public static Command moveElevator(Elevator elevator, double spd) {
    return Commands.runOnce(
        () -> {
          elevator.manualMove(spd * 0.25);
          elevator.setElevatorHeight(elevator.getElevatorHeight() + 1);
        },
        elevator);
  }

  public static Command moveWrist(Wrist wrist, double spd) {
    return Commands.runOnce(
        () -> {
          wrist.moveWrist(spd * 0.25);
          wrist.setWristAngle(wrist.getAngle() + 1);
        },
        wrist);
  }
}
