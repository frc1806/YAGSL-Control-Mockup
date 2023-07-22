
package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.shuffleboard.tabs.tabsUtil.XboxControllerConfigValues;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.SWATXboxController;

public class DriverControls extends SubsystemBase {
    private SWATXboxController driverController;
    private SWATXboxController operatorController;
    private SWATXboxController debugController;
    DriverControlType selectedControls;
    DoubleSupplier robotHeadingSupplier;

    double driverWantedRotationAngle;
    double driverWantedTranslationAngle;
    boolean wasHeadingStickOutsideDeadzone;
    boolean wasTranslationStickOutsideDeadzone;

    public static SendableChooser<DriverControlType> controllerConfigChooser;

    private enum DriverControlType {
        Classic,
        Forza,
    }

    /**
     * Creates a DriverControls subsystem. The subsystem used to keep track of the
     * driver's controls based on the status of a sendable chooser.
     */
    public DriverControls(DoubleSupplier robotHeadingSupplier) {
        driverController = new SWATXboxController(Constants.kDriverPort, "Driver",
                XboxControllerConfigValues.kDriverControllerDefaultConfig);
        operatorController = new SWATXboxController(Constants.kOperatorPort, "Operator",
                XboxControllerConfigValues.kOperatorControllerDefaultConfig);
        debugController = new SWATXboxController(Constants.kDebugPort, "Debug",
                XboxControllerConfigValues.kOperatorControllerDefaultConfig);

    }

    /**
     * Get the current selected driver controls
     */
    public DriverControlType getCurrentDriverControls() {
        return selectedControls;
    }

    public Translation2d getRadDriveWantedTranslation(){
        return getWantedTransalation(driverController.getLeftY(), -driverController.getLeftX(), driverController.getRightTriggerAxis());
        
    }

    private Translation2d getWantedTransalation(double joystickFieldX, double joystickFieldY,double throttle){
        updateDriverWantedTranslationAngle(joystickFieldX, joystickFieldY);
        if(throttle == 0.0) return new Translation2d();
        double x = throttle * Math.cos(driverWantedTranslationAngle);
        double y = throttle * Math.sin(driverWantedTranslationAngle);
        return new Translation2d(x, y);
    }

    private void updateDriverWantedTranslationAngle(double joystickFieldX, double joystickFieldY){
        if(isRobotTranslationJoystickOutsideDeadzone())
        {
            driverWantedTranslationAngle = getWantedAngleFromJoystick(joystickFieldX, joystickFieldY);
            wasTranslationStickOutsideDeadzone = true;
        }
        else if(wasTranslationStickOutsideDeadzone)
        {
            driverWantedTranslationAngle = getNearest45inRadians(driverWantedTranslationAngle);
            wasTranslationStickOutsideDeadzone = false;
        }
    }

    private boolean isRobotTranslationJoystickOutsideDeadzone(){
        return Math.hypot(driverController.getLeftX(), driverController.getLeftY()) > 0.6;
    }

    public double getWantedRadDriveRobotAngle(){
        return driverWantedRotationAngle / Math.PI;
    }


    private boolean isRobotAngleJoystickOutsideDeadzone(){
        return Math.hypot(driverController.getRightX(), driverController.getRightY()) > 0.8;
    }

    private double getCurrentHeadingJoystickAngle(){
        return getWantedAngleFromJoystick(driverController.getRightY(), -driverController.getRightX());
    }

    private double getWantedAngleFromJoystick(double joystickFieldX, double joystickFieldY){
        double xyMag = Math.hypot(joystickFieldX, joystickFieldY);
        double angle = Math.acos(joystickFieldX / xyMag) * (joystickFieldY > 0? 1:-1);
        return angle;
    }



    /**
     * Get drivetrain throttle control value
     * 
     * @return a {@link double} between -1 and 1
     */


    // Operator Controls

    // Operator LED Control

    // Debug Controls


    /*
    public double d_cymbalThrottle() {
        return debugController.getLeftY();
    }
    

    public boolean d_wantCymbalManual() {
        return d_cymbalThrottle() != 0;
    }
    */

    // Debug Tabs

    public boolean debugTabs() {
        return driverController.getStartButton() && driverController.getBackButton();
    }

    public boolean o_debugTabs() {
        return operatorController.getStartButton() && operatorController.getBackButton();
    }

    public boolean d_debugTabs() {
        return debugController.getStartButton() && debugController.getBackButton();
    }

    private double getNearest45inRadians(double input){
        double fourtyFive = Math.PI / 4.0;
        return Math.round(input / fourtyFive) * fourtyFive;
    }



    /**
     * Register all the controls for the robot. Note that selected controls updates won't happen without a roborio reboot due to the way that triggers work.
     * @param driveTrain Our one and only drivetrain
     * @param visionSubsystem our (currently) one and only vision subsystem representing the limelight
     */
    public void registerTriggers(SwerveSubsystem driveTrain){
        //D

    }

    @Override
    public void periodic() {
        boolean outOfDeadzone = isRobotAngleJoystickOutsideDeadzone();
        if(DriverStation.isDisabled()){
            driverWantedRotationAngle = robotHeadingSupplier.getAsDouble() % (2* Math.PI);
        }
        else{
            if(outOfDeadzone)
            {
                driverWantedRotationAngle = getCurrentHeadingJoystickAngle();
            }
            else if (wasHeadingStickOutsideDeadzone)
            {
                driverWantedRotationAngle = getNearest45inRadians(driverWantedRotationAngle);
            }
        }

        wasHeadingStickOutsideDeadzone = outOfDeadzone;
        driverController.updateConfig();
        System.out.println("Rot:" + Units.radiansToDegrees(driverWantedRotationAngle));
    }
   

}
