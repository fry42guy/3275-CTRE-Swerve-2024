package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.TargetCalcs2;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    
    
 /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
 private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
 /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
 private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

 private boolean hasAppliedOperatorPerspective = false;
   

      private final Field2d m_feild = new Field2d();

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()->isRedAlliance(), // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public boolean isRedAlliance()
    {
        if (DriverStation.getAlliance().get() == Alliance.Red)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
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

    public void init(){
       
    }

    public void periodic(){
        
m_feild.setRobotPose(getrobotpose());
        SmartDashboard.putData("Field", m_feild);
//m_feild.setRobotPose(LimelightHelpers.getBotPose2d("limelight"));

//Updateoperatorperspective();
       






    }


    public Pose2d getrobotpose(){

return m_odometry.getEstimatedPosition();


    }

    public void LimelightUpdatePose(){
        LimelightHelpers.PoseEstimate limelightMeasurement;
        if (isRedAlliance() ==true) {
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight");

        }
        else {
            limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        }
        
     
       // //SmartDashboard.putString("BluePOS", LimelightHelpers.getBotPose2d_wpiBlue("limelight").);
     
       if(limelightMeasurement.tagCount >= 1)
       {
         this.setVisionMeasurementStdDevs(VecBuilder.fill(1,1,1));//.7,.7,9999999));
         this.addVisionMeasurement(
             limelightMeasurement.pose,
             limelightMeasurement.timestampSeconds);
     
             
         
       }
     
    }

public Rotation2d Getoffsetroation(){

   return this.m_fieldRelativeOffset;
}



//  public void Updateoperatorperspective(){

//     if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
//         DriverStation.getAlliance()
//             .ifPresent(
//                 (allianceColor) -> {
//                   this.setOperatorPerspectiveForward(
//                       allianceColor == Alliance.Red
//                           ? RedAlliancePerspectiveRotation
//                           : BlueAlliancePerspectiveRotation);
//                   hasAppliedOperatorPerspective = true;
//                 });
//             }}


}
