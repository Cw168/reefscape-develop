// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgeaCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorWristCommands;
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

  private boolean intakeToggle = false;

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
    NamedCommands.registerCommand("L2", new SetWristAndElevator(this, 1));
    NamedCommands.registerCommand("L3", new SetWristAndElevator(this, 2));
    NamedCommands.registerCommand("LH", new SetWristAndElevator(this, 8));
    NamedCommands.registerCommand("L0", new SetWristAndElevator(this, 0));
    NamedCommands.registerCommand("L2A", new SetWristAndElevator(this, 60));
    NamedCommands.registerCommand("L3A", new SetWristAndElevator(this, 7));
    NamedCommands.registerCommand("Intake", IntakeCommands.intake(wrist));
    NamedCommands.registerCommand("StopIntake", IntakeCommands.stop(wrist));
    NamedCommands.registerCommand("Outake", IntakeCommands.outake(wrist));
    NamedCommands.registerCommand("IntakeL2", ElevatorWristCommands.setWristLevel(wrist, 1));
    NamedCommands.registerCommand("IntakeL3", ElevatorWristCommands.setWristLevel(wrist, 2));
    NamedCommands.registerCommand("IntakeHuman", ElevatorWristCommands.setWristLevel(wrist, 3));
    NamedCommands.registerCommand("IntakeAlgae", ElevatorWristCommands.setWristLevel(wrist, 4));
    // NamedCommands.registerCommand("Flywheel", AlgeaCommands.shoot(shooter, ));

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
    // c_controller2.x;
    // Lock to 0° when A button is held
    // used to align for shooting
    /*c_controller2
    .x()
    .whileTrue(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> new Rotation2d()));*/

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
    // Tare swerve pos
    /*controller
    .y()
    .onTrue(
        Command.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));*/

    // controller.y().onTrue(new SetWristAndElevator(this, 0));
    // level 1 state, depend on is coral loaded

    // intake

    // transfer
    controller.rightBumper().onTrue(AlgeaCommands.Transfer(shooter, 0.5));
    controller.rightBumper().onFalse(AlgeaCommands.Transfer(shooter, 0));

    // shoot
    controller.rightTrigger().onTrue(AlgeaCommands.shoot(shooter, true));
    controller.rightTrigger().onFalse(AlgeaCommands.shoot(shooter, false));
    // controller.y().onTrue(Drive.stopWithX());
    // intake
    controller.leftBumper().onTrue(IntakeCommands.intake(wrist));
    controller.leftBumper().onFalse(IntakeCommands.stop(wrist));
    controller.leftTrigger().onTrue(IntakeCommands.outake(wrist));
    controller.leftTrigger().onFalse(IntakeCommands.stop(wrist));
    /*
    // elevator and wrist
    c_controller2.povDown().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 0));
    c_controller2.povLeft().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 1));
    c_controller2.povUp().onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 2));
    /*c_controller2
        .povRight()
        .onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 3));
    c_controller2
        .leftBumper()
        .onTrue(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 4));*/

    // manuel elevator
    c_controller2.y().onTrue(ElevatorWristCommands.moveElevator(elevator, -0.5));
    c_controller2.y().onFalse(ElevatorWristCommands.moveElevator(elevator, 0));
    c_controller2.a().onTrue(ElevatorWristCommands.moveElevator(elevator, 0.5));
    c_controller2.a().onFalse(ElevatorWristCommands.moveElevator(elevator, 0));

    // manuel wrist
    c_controller2.b().onTrue(ElevatorWristCommands.moveWrist(wrist, -1));
    c_controller2.b().onFalse(ElevatorWristCommands.stopWrist(wrist));
    c_controller2.x().onTrue(ElevatorWristCommands.moveWrist(wrist, 1));
    c_controller2.x().onFalse(ElevatorWristCommands.stopWrist(wrist));
    controller
        .a()
        .onTrue(
            (ElevatorWristCommands.moveWrist(wrist, -1)
                    .withTimeout(0.1)
                    .andThen(ElevatorWristCommands.moveWrist(wrist, 1).withTimeout(0.1))
                    .andThen(() -> intakeToggle = false)
                    .repeatedly())
                .onlyIf(() -> intakeToggle == true)
                .andThen((ElevatorWristCommands.stopElevator(elevator)))
                .andThen(() -> intakeToggle = true)
                .onlyIf(() -> intakeToggle == false));

    // controller.a().toggleOnFalse(ElevatorWristCommands.stopWrist(wrist));
    /*
    // funnel
    controller.leftBumper().onTrue(FunnelCommands.FunnelUp(funnel));
    controller.leftTrigger().onTrue(FunnelCommands.FunnelDown(funnel));*/

    // wrist
    controller
        .povDown()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 0)
                .andThen(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 0)));
    controller
        .povLeft()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 1)
                .andThen(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 1)));
    controller
        .povUp()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 2)
                .andThen(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 2)));
    controller
        .povRight()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 3)
                .andThen(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 8)));

    c_controller2
        .povDown()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 4)
                .andThen(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 6)));

    c_controller2
        .povUp()
        .onTrue(
            ElevatorWristCommands.setWristLevel(wrist, 4)
                .andThen(ElevatorWristCommands.setElevatorWristStage(elevator, wrist, 7)));
    // ElevatorWristCommands.c_controller2.getRightY();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    autoSystem.add("A", autoChooser.get());
    Command autonomous = autoChooser.get();

    return autonomous;
  }
}
