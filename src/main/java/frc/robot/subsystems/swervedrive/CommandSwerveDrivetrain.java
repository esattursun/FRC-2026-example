package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.util.Reef.coralStationConstants.Station;
import swervelib.math.SwerveMath;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {4
    private SparkMax x;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();


//       /**
//    * April Tag Field Layout of the year.
//    */
//   public static final AprilTagFieldLayout fieldLayout                     = AprilTagFieldLayout.loadField(
//       AprilTagFields.k2025ReefscapeAndyMark);

  public Field2d field = new Field2d();

      
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    //
    public final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */

    //
    public final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

      /**
   * Enable vision odometry updates while driving.
   */
  private final boolean             visionDriveTest     = false;
   /**
   * PhotonVision class to keep an accurate odometry.
   */
  public Vision vision;

  private int value= 0;


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        if (visionDriveTest)
        {
          setupPhotonVision();
          // Stop the odometry thread if we are using vision that way we can synchronize updates better.
        //   swerveDrive.stopOdometryThread();
        super.getOdometryThread().stop();
        }
    }

    
  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision(){
    vision = new Vision(this::getPose, field);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

    public Pose2d getPose(double timeSeconds) {
    Pose2d currPose = this.getPose();
    // Rotation2d currRotation = currPose.getRotation();
    ChassisSpeeds speeds = getState().Speeds;
    double velocityX = speeds.vxMetersPerSecond;
    double velocityY = speeds.vyMetersPerSecond;

    double transformX = timeSeconds * velocityX;
    double transformY = timeSeconds * velocityY;
    Rotation2d transformRotation = new Rotation2d(timeSeconds * speeds.omegaRadiansPerSecond);
    Transform2d transformPose = new Transform2d(transformX, transformY, transformRotation);
    Pose2d predictedPose = currPose.plus(transformPose);

    // DogLog.log("Predicted Pose", predictedPose);

    return predictedPose;
  }
  
  
//   public Pose2d getx() {
//     return super.;
//   }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });

            if (visionDriveTest){
                super.getOdometryThread().start();
                // swerveDrive.updateOdometry();
                vision.updatePoseEstimation(this);
          
          
          }
        }
        field.setRobotPose(getPose());
        SmartDashboard.putData("field", field);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurem    ent standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }


    public void driveFieldOriented(ChassisSpeeds velocity){
        double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
   
   
       SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
               .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
               .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
   
           this.applyRequest(()-> 
           drive.withVelocityX(velocity.vxMetersPerSecond).
           withVelocityY(velocity.vyMetersPerSecond).
           withRotationalRate(velocity.omegaRadiansPerSecond));
       }




    public void drive(ChassisSpeeds velocity){
     double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
     double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);



    SwerveRequest.RobotCentric robotOriented = new SwerveRequest.RobotCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.applyRequest(()-> 
        robotOriented.withVelocityX(velocity.vxMetersPerSecond).
        withVelocityY(velocity.vyMetersPerSecond).
        withRotationalRate(velocity.omegaRadiansPerSecond));
    }

    public void changevalue(int value){
        this.value = value;
      }
      public int getvalue(){
        return this.value;
      }

  public Station getclosestCoralStation(){
    double[] distances = {
      getDistanceToTranslation(Station.AB.getStationX(), Station.AB.getStationY()),    // AB
      getDistanceToTranslation(Station.CD.getStationX(), Station.CD.getStationY()),    // CD
      getDistanceToTranslation(Station.EF.getStationX(), Station.EF.getStationY()),    // EF
      getDistanceToTranslation(Station.GH.getStationX(), Station.GH.getStationY()),    // GH
      getDistanceToTranslation(Station.IJ.getStationX(), Station.IJ.getStationY()),    // IJ
      getDistanceToTranslation(Station.KL.getStationX(), Station.KL.getStationY())     // KL
  };
  String[] names = {"ab", "cd", "ef", "gh", "ij", "kl"};
  int minIndex = findMinDistanceIndex(distances);
  SmartDashboard.putString( "Ismi: " , names[minIndex]);

  return Station.values()[minIndex];
  }

   public double getDistanceToTranslation(double x, double y){

    Translation2d robotTranslation =getPose().getTranslation();
    Translation2d CoralStationTranslation = new Translation2d(x,y);//3.30 3.9 , 3.9, 3

    double coralStationDistance = robotTranslation.getDistance(CoralStationTranslation);
    return coralStationDistance;
  }

  public static int findMinDistanceIndex(double[] distances) {
    int minIndex = 0;
    for (int i = 1; i < distances.length; i++) {
        if (distances[i] < distances[minIndex]) {
            minIndex = i;
        }
    }
    return minIndex;
}
public Station getclosestCoralStationx(){
    double[] distances = {
      getDistanceToTranslation(Station.AB.getStationXx(), Station.AB.getStationYy()),    // AB
      getDistanceToTranslation(Station.CD.getStationXx(), Station.CD.getStationYy()),    // CD
      getDistanceToTranslation(Station.EF.getStationXx(), Station.EF.getStationYy()),    // EF
      getDistanceToTranslation(Station.GH.getStationXx(), Station.GH.getStationYy()),    // GH
      getDistanceToTranslation(Station.IJ.getStationXx(), Station.IJ.getStationYy()),    // IJ
      getDistanceToTranslation(Station.KL.getStationXx(), Station.KL.getStationYy())     // KL
  };
 
  String[] names = {"ab", "cd", "ef", "gh", "ij", "kl"};
  int minIndex = findMinDistanceIndex(distances);
  SmartDashboard.putString( "Ismi: " , names[minIndex]);

  return Station.values()[minIndex];
}
/**
 * @param vertical The vertical change relative to the Coral Station center.
 * @param horizontal The horizontal change relative to the Coral Station center.
 * @return The true position of the Coral Station as a {@link Pose2d}.
 */
public Pose2d gettrueCoralStationPose(double vertical,double horizontal ){
    Station station = getclosestCoralStation();
    
    Pose2d coralStationpose2 =Vision.getAprilTagPose(station.getStationTagId(), new Transform2d(new Translation2d(),new Rotation2d()));
    Pose2d trueCoralStationPose = coralStationpose2.plus( new Transform2d(vertical, horizontal, Rotation2d.fromDegrees(180)));
     

        return trueCoralStationPose;     
    }

/**
 * @param vertical The vertical change relative to the Coral Station center.
 * @param horizontal The horizontal change relative to the Coral Station center.
 * @return The true position of the Coral Station as a {@link Pose2d}.
 */
public Pose2d gettrueCoralStationPose(Station wantedstation,double vertical,double horizontal ){
    Station station = wantedstation;
  
    Pose2d coralStationpose2 =Vision.getAprilTagPose(station.getStationTagId(), new Transform2d(new Translation2d(),new Rotation2d()));
      Pose2d trueCoralStationPose = coralStationpose2.plus( new Transform2d(vertical, horizontal, Rotation2d.fromDegrees(180)));
       
        return trueCoralStationPose;     
    }
    public Pose2d gettrueCoralStationPosex(double vertical,double horizontal ){
      Station station = getclosestCoralStationx();
      
      Pose2d coralStationpose2 =Vision.getAprilTagPose(station.getStationTagIdx(), new Transform2d(new Translation2d(),new Rotation2d()));
      Pose2d trueCoralStationPose = coralStationpose2.plus( new Transform2d(vertical, horizontal, Rotation2d.fromDegrees(180)));
       
  
          return trueCoralStationPose;     
      }
      public Pose2d getnetpose(double vertical ){
      
        Pose2d coralStationpose2 =Vision.getAprilTagPose(isRedAlliance()?5 : 14 , new Transform2d(new Translation2d(),new Rotation2d()));
        Pose2d trueCoralStationPose = coralStationpose2.plus( new Transform2d(vertical, 0.5, Rotation2d.fromDegrees(180)));
         
    
            return trueCoralStationPose;
        }
  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }




  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return SwerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getPose().getRotation().getRadians(),
                                                        TunerConstants.kSpeedAt12VoltsMps);
  }



  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    if(fieldRelative){
     drive(new ChassisSpeeds(translation.getX(),
        translation.getY(),
              rotation));
    }else{
        driveFieldOriented(new ChassisSpeeds(translation.getX(),
        translation.getY(),
              rotation));
    }
}

   public void brake(){
    this.applyRequest(()-> new SwerveRequest.SwerveDriveBrake());

   }
}