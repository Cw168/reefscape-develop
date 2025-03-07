// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.SuperStructureState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristAndElevator extends Command {

  RobotContainer robot;
  int level = 0;

  /** Creates a new SetWristAndElevator. */
  public SetWristAndElevator(RobotContainer robot, int level) {
    addRequirements(robot.wrist, robot.elevator);

    this.robot = robot;
    this.level = level;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (level) {
      case 1: // Ground
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_L1;
        SmartDashboard.putString("Elevator", "Ground");
        break;
      case 2: // L1
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_L2;
        SmartDashboard.putString("Elevator", "L1 Coral");
        break;
      case 3: // L2
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_L3;
        SmartDashboard.putString("Elevator", "L2 Coral");
        break;
      case 4: // L3
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_L4;
        SmartDashboard.putString("Elevator", "L2 Alge");
        break;
      case 5: // Transfer
        RobotContainer.groundIntake = true;
        robot.targetState = SuperStructureState.STATE_TRANSFER;
        SmartDashboard.putString("Elevator", "Transfer");
        break;
      case 6: // L1 Ball
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_L1B;
        SmartDashboard.putString("Elevator", "L1 Alge");
        break;
      case 7: // L1 Ball
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_L2B;
        SmartDashboard.putString("Elevator", "L2 Alge");
        break;
      default:
        RobotContainer.groundIntake = false;
        robot.targetState = SuperStructureState.STATE_SOURCE;
        break;
    }
    robot.elevator.setElevatorHeight(robot.targetState.height);
    // robot.wrist.setWristAngle(robot.targetState.angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robot.elevator.setElevatorHeight(robot.targetState.height);
    // robot.wrist.setWristAngle(robot.targetState.angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robot.elevator.isDone().getAsBoolean();
    // return isFinished;

  }
}
