// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgeaCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorWristCommands;
import frc.robot.commands.FunnelCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.SetWristAndElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Funnel;
import frc.robot.subsystems.elevator.Wrist;
import frc.robot.subsystems.intake.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final LimeLight vision;
  // public final AlgeaIntake intake;
  public final Shooter shooter;
  public final Wrist wrist;
  public final Elevator elevator;
  public final Funnel funnel;
  public SuperStructureState currentState = SuperStructureState.STATE_SOURCE;
  public SuperStructureState targetState = SuperStructureState.STATE_SOURCE;

  // Controller
  public final XboxController m_controller = new CommandXboxController(0).getHID();
  public final CommandXboxController controller = new CommandXboxController(0);
  public final XboxController controller2 = new XboxController(1);
  public final CommandXboxController c_controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public static boolean groundIntake = true;
  ShuffleboardTab autoSystem;

  public Drive getDrive() {
    return drive;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // intake = new AlgeaIntake();
    shooter = new Shooter();
    wrist = new Wrist();
    elevator = new Elevator();
    funnel = new Funnel();
    // Real robot, instantiate hardware IO implementations
    // vision = new LimeLight();
    // wrist = new Wrist();
    // elevator = new Elevator();
    // intake = new Intake();
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoSystem = Shuffleboard.getTab("Auto");

    NamedCommands.registerCommand("L0", new SetWristAndElevator(this, 0));
    NamedCommands.registerCommand("L1", new SetWristAndElevator(this, 1));
    NamedCommands.registerCommand("L2", new SetWristAndElevator(this, 2));
    NamedCommands.registerCommand("L3", new SetWristAndElevator(this, 3));
    NamedCommands.registerCommand("L4", new SetWristAndElevator(this, 4));
    NamedCommands.registerCommand("L5", new SetWristAndElevator(this, 5));
    NamedCommands.registerCommand("L6", new SetWristAndElevator(this, 6));
    NamedCommands.registerCommand("L7", new SetWristAndElevator(this, 7));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // used to align for shooting
    c_controller2
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Point wheels in x formation to stop
    //  controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Point robot to april tag
    // controller
    //     .leftStick()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> controller.getLeftY(),
    //             () -> controller.getLeftX(),
    //             () -> new Rotation2d(Units.degreesToRadians(vision.autoRotate()))));

    // Align robot to april tag
    /*controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> vision.autoTranslateY(),
                () -> vision.autoTranslateX(),
                () -> new Rotation2d(Units.degreesToRadians(vision.autoRotate()))));
    */
    // Reset gyro
    // controller
    //     .y()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.y().onTrue(new SetWristAndElevator(this, 0));
    // level 1 state, depend on is coral loaded

    // intake

    // transfer
    controller.rightTrigger().onTrue(AlgeaCommands.Transfer(shooter, 0.1));
    controller.rightTrigger().onFalse(AlgeaCommands.Transfer(shooter, 0));
    controller.axisMagnitudeGreaterThan(3, 0.1).onFalse(AlgeaCommands.Transfer(shooter, 0));

    // shoot
    controller.rightBumper().onTrue(AlgeaCommands.shoot(shooter, true));
    controller.rightBumper().onFalse(AlgeaCommands.shoot(shooter, false));

    // intake
    controller.a().onTrue(IntakeCommands.intake(wrist));
    controller.a().onFalse(IntakeCommands.stop(wrist));
    controller.b().onTrue(IntakeCommands.outake(wrist));
    controller.b().onFalse(IntakeCommands.stop(wrist));

    // elevator and wrist
    controller.povDown().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 0));
    controller
        .povDownLeft()
        .onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 1));
    controller.povLeft().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 2));
    controller.povUpLeft().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 3));
    controller.povUp().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 4));
    controller.povUpRight().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 5));
    controller.povRight().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 6));
    controller
        .povDownRight()
        .onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 7));

    // manuel elevator
    c_controller2
        .axisMagnitudeGreaterThan(1, 0.1)
        .whileTrue(ElevatorWristCommands.moveElevator(elevator, c_controller2.getLeftY()));
    c_controller2
        .axisMagnitudeGreaterThan(1, 0.1)
        .onFalse(ElevatorWristCommands.moveElevator(elevator, 0));

    // manuel wrist
    c_controller2.rightTrigger().onTrue(ElevatorWristCommands.moveWrist(wrist, 1));
    c_controller2.rightTrigger().onFalse(ElevatorWristCommands.stopWrist(wrist));

    c_controller2.rightBumper().onTrue(ElevatorWristCommands.moveWrist(wrist, -1));
    c_controller2.rightBumper().onFalse(ElevatorWristCommands.stopWrist(wrist));

    // funnel
    controller.leftBumper().onTrue(FunnelCommands.FunnelUp(funnel));
    controller.leftTrigger().onTrue(FunnelCommands.FunnelDown(funnel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    autoSystem.add("A", autoChooser.get());
    Command autonomous =
        AlgeaCommands.positionIntake(wrist)
            .andThen(new WaitCommand(1))
            .andThen(new WaitCommand(1))
            // .andThen(new SetWristAndElevator(this, 3).withTimeout(2))
            .andThen(autoChooser.get())
            .andThen(IntakeCommands.intake(wrist).withTimeout(1))
            .andThen(new WaitCommand(1));

    return autonomous;
  }
}
