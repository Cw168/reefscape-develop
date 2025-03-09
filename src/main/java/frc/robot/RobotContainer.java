// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetWristAndElevator;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
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
    // controller
    //     .x()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

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

    controller
        .povDown()
        .onTrue(
            new ConditionalCommand(
                new SetWristAndElevator(this, 1),
                new SetWristAndElevator(this, 5),
                () -> groundIntake)); // Ground intake
    // level 2 state, depend on is coral loaded
    controller.povLeft().onTrue(new SetWristAndElevator(this, 2)); // L1
    // level 3 state, depend on is coral loaded
    controller.povUp().onTrue(new SetWristAndElevator(this, 3)); // L2 and L1 Outtake
    // level 4 state, depend on is coral loaded
    controller.povRight().onTrue(new SetWristAndElevator(this, 7)); // L2 Outtake

    c_controller2.povDown().onTrue(new SetWristAndElevator(this, 1)); // Ground
    // level 2 state, depend on is coral loaded
    c_controller2.povLeft().onTrue(new SetWristAndElevator(this, 5)); // Transfer
    // level 3 state, depend on is coral loaded
    c_controller2.povUp().onTrue(new SetWristAndElevator(this, 2)); // L1
    // level 4 state, depend on is coral loaded
    c_controller2.povRight().onTrue(new SetWristAndElevator(this, 3)); // L2
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    autoSystem.add("A", autoChooser.get());
    Command autonomous =
             CoralCommands.positionIntake(wrist)
            .andThen(new WaitCommand(1))
            .andThen(CoralCommands.intakeBall(shooter).withTimeout(1))
            .andThen(new WaitCommand(1))
            //.andThen(new SetWristAndElevator(this, 3).withTimeout(2))
            .andThen(autoChooser.get())
            .andThen(CoralCommands.outake(shooter).withTimeout(1))
            .andThen(CoralCommands.moveIntake(wrist).withTimeout(1))
            .andThen(new WaitCommand(1));

    return autonomous;
  }
}
