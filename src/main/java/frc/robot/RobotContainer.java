// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.upsystemCommands.AutoActuation;
import frc.robot.commands.upsystemCommands.DelayedScore;
import frc.robot.commands.upsystemCommands.HardScore;
import frc.robot.commands.upsystemCommands.Score;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.Wriststates;
import frc.robot.subsystems.catcher_left;
import frc.robot.subsystems.catcher_right;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;
import frc.util.Actuation.ActuationConstans.upsystemstates;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final static CommandXboxController joystick = new CommandXboxController(0);
    private final static CommandXboxController joystick2 = new CommandXboxController(1);
    
            public final static catcher_right CATCHER_RIGHT = new catcher_right();
        public final static catcher_left CATCHER_LEFT = new catcher_left();
    
        public final static Wrist PIVOT = new Wrist(()-> MathUtil.applyDeadband(joystick2.getRightY(),0.05));
    
        public final static Elevator ELEVATOR = new Elevator(()-> joystick2.getLeftY());

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        configureBindings();


      CATCHER_RIGHT.setDefaultCommand(Commands.run(()-> CATCHER_RIGHT.setmotor(0), CATCHER_RIGHT));
      CATCHER_LEFT.setDefaultCommand(Commands.run(()-> CATCHER_LEFT.setmotor(0), CATCHER_LEFT));

      PIVOT.setDefaultCommand(Commands.run(()-> PIVOT.setState(Wriststates.MANUAL), PIVOT));

      ELEVATOR.setDefaultCommand(Commands.run(()-> ELEVATOR.setState(ElevatorStates.MANUAL), ELEVATOR));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 1) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed *1       ) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate *.8   ) // Drive counterclockwise with negative X (left)
            )
        );

        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.rightTrigger(0.4).whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // joystick.leftTrigger(.04).whileTrue(Commands.run(
        //     ()-> drivetrain.setControl(
        //         robotOriented.withVelocityX(-joystick.getLeftY()).
        //         withVelocityY(-joystick.getLeftX()).
        //         withRotationalRate(-joystick.getRightX())))); 
        


         joystick.leftTrigger(.04).whileTrue(drivetrain.applyRequest(() ->
            
                robotOriented.withVelocityX(-joystick.getLeftY()).
                withVelocityY(-joystick.getLeftX()).
                withRotationalRate(-joystick.getRightX()* 2)));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);





        
        joystick2.x().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l3));
        joystick2.a().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l4));
        joystick2.b().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.l2));
        joystick2.y().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.source));

        joystick2.povDown().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.underalgae));
        joystick2.povUp().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.upperalgae));
        joystick2.povRight().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.lb2));
        joystick2.povLeft().whileTrue(new AutoActuation(ELEVATOR, PIVOT, upsystemstates.net));

        joystick2.rightTrigger(0.06).whileTrue(Commands.run(()-> CATCHER_RIGHT.setrightmotor(()->joystick2.getRightTriggerAxis()), CATCHER_RIGHT));
        joystick2.leftTrigger(0.06).whileTrue(Commands.run(()-> CATCHER_LEFT.setleftmotor(()->-joystick2.getLeftTriggerAxis()), CATCHER_LEFT));

        joystick2.rightBumper().whileTrue( new Score(CATCHER_LEFT, CATCHER_RIGHT));
        joystick2.back().whileTrue(new DelayedScore(CATCHER_LEFT, CATCHER_RIGHT));
        joystick2.leftBumper().whileTrue( new HardScore(CATCHER_LEFT, CATCHER_RIGHT));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
