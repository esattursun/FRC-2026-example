package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.CommandSwerveDrivetrain;

public class AimatCoralStation extends Command {
  private final CommandSwerveDrivetrain drivebase;
  private double rotation = 0;

  public AimatCoralStation(CommandSwerveDrivetrain drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivebase.changevalue(3);
      rotation = drivebase.getclosestCoralStation().getRotationCoralStation();
    
    double leftY = RobotContainer.joystick.getLeftY();
    double leftX = RobotContainer.joystick.getLeftX();
    Rotation2d rotation2d = Rotation2d.fromDegrees(rotation);
    
    ChassisSpeeds targetSpeeds = drivebase.getTargetSpeeds(leftY, leftX, rotation2d);
    
    if (Robot.isSimulation()) {
      drivebase.driveFieldOriented(new ChassisSpeeds(
        targetSpeeds.vxMetersPerSecond,
        targetSpeeds.vyMetersPerSecond,
        targetSpeeds.omegaRadiansPerSecond * 2.8
      ));
    } else {
      double omega = targetSpeeds.omegaRadiansPerSecond * 5;
      double adjustedOmega = omega > 0 ? Math.min(1, omega) : Math.max(-1, omega);
      
      drivebase.driveFieldOriented(new ChassisSpeeds(
        targetSpeeds.vxMetersPerSecond,
        targetSpeeds.vyMetersPerSecond,
        adjustedOmega
      ));

      SmartDashboard.putNumber("rott", omega);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
