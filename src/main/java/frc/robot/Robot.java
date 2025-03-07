// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  double curAngle = 0;
  double curHeight = 0;
  int curWristState = 0;
  boolean moving = false;
  boolean inAuto = true;

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // robotContainer.intake.intake(robotContainer.m_controller.getAButton());
    if (robotContainer.m_controller.getRightTriggerAxis() > 0.01) {
      robotContainer.shooter.shoot(true);
    } else if (!inAuto) {
      robotContainer.shooter.shoot(false);
    }
    robotContainer.shooter.shoot(robotContainer.m_controller.getRightBumperButton());

    if (robotContainer.m_controller.getLeftTriggerAxis() > 0.01) {
      robotContainer.shooter.intake(1);
    } else if (robotContainer.m_controller.getLeftBumperButton()) {
      robotContainer.shooter.intake(-1);
    } else if (!inAuto) {
      robotContainer.shooter.intake(0);
    }

    if (robotContainer.controller2.getXButton()) { // Up
      // curAngle += 5;
      // robotContainer.wrist.setWristAngle(curAngle);
      robotContainer.wrist.moveWrist(0.25);
    } else if (robotContainer.controller2.getYButton()) {
      // curAngle -= 5;
      // robotContainer.wrist.setWristAngle(curAngle);
      robotContainer.wrist.moveWrist(-0.25);
    } else if (!inAuto) {
      curAngle = robotContainer.wrist.getAngle();
      // robotContainer.wrist.setWristAngle(curAngle + 1);
    }
    SmartDashboard.putBoolean("A", inAuto);

    if (robotContainer.m_controller.getYButtonPressed()
        || robotContainer.controller2.getAButtonPressed()) {
      curWristState = (curWristState + 1) % 3;
    }
    switch (curWristState) {
      case 0:
        if (!inAuto) {
          robotContainer.wrist.setWristAngle(-70);
          SmartDashboard.putString("Intake Angle", "L3");
          break;
        }
      case 1:
        robotContainer.wrist.setWristAngle(-100);
        SmartDashboard.putString("Intake Angle", "L1/2");
        break;
      case 2:
        robotContainer.wrist.setWristAngle(-170);
        SmartDashboard.putString("Intake Angle", "Source");
        break;

      default:
        break;
    }

    if (robotContainer.m_controller.getAButton()) { // Intake
      robotContainer.wrist.moveIntake(1);
    } else if (robotContainer.m_controller.getBButton()) {
      robotContainer.wrist.moveIntake(-1);
    } else if (!inAuto) {
      robotContainer.wrist.moveIntake(0);
    }
    robotContainer.shooter.transferSpeed(robotContainer.m_controller.getRightTriggerAxis());
    if (Math.abs(robotContainer.controller2.getLeftY()) > 0.1) {
      robotContainer.elevator.manualMove(robotContainer.controller2.getLeftY() * 0.25);
      moving = true;
    } else if (moving) {
      moving = false;
      robotContainer.elevator.manualMove(0);
      robotContainer.elevator.setElevatorHeight(robotContainer.elevator.getElevatorHeight());
    }
    /*
    if (robotContainer.controller2.getLeftY() > 0.5) {
      curHeight -= 0.25;
      robotContainer.elevator.setElevatorHeight(curHeight);
    } else if (robotContainer.controller2.getLeftY() < -0.5) {
      curHeight += 0.25;
      robotContainer.elevator.setElevatorHeight(curHeight);
    } else {
      curHeight = robotContainer.elevator.getElevatorHeight();
    }
       */
    // curHeight = robotContainer.elevator.get

    robotContainer.shooter.shoot(robotContainer.m_controller.getRightBumperButton());
    SmartDashboard.putBoolean("Mode", RobotContainer.groundIntake);
    // if (robotContainer.m_controller.getRightBumperButtonPressed())
    // robotContainer.elevator.stop();
    // int t = robotContainer.m_controller.getPOV();
    // System.out.println(t);
    // robotContainer.elevator.setElevatorHeight(15);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    inAuto = true;
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    inAuto = false;
    // robotContainer.elevator.currentState = SuperStructureState.STATE_SOURCE;
    // robotContainer.wrist.currentState = SuperStructureState.STATE_SOURCE;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    // robotContainer.elevator.resetPosition();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public boolean isXPressed() {
    return robotContainer.m_controller.getXButton();
  }

  public boolean isYPressed() {
    return robotContainer.m_controller.getYButton();
  }
}
