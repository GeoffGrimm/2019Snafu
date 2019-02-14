
package frc.robot;

import java.text.SimpleDateFormat;
import java.util.Date;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* encoder -> 3 pin (3) & 2 pin (2) -> ribbon cable (R) -> PWMs (PA, PB)
B -> 3Bk -> RGn -> PBWt
5 -> 3Rd -> RRd -> PARd & PBRd
A -> 3Wt -> RWt -> PAWt
X -> do not use
G -> 2Bk -> RBk -> PABk & PBBk
*/

public class Robot extends TimedRobot {

  public XboxController xbox;

  public Encoder encoder;

  public Spark testController, elevatorController, coneController, fingerController;
  public WPI_TalonSRX rearLeft, frontLeft, frontRight, rearRight;
  MecanumDrive mecanum;
  
  public static final double ConePowerDefault = 0.75;

  public static final double FingersPowerDefault = 0.50;
  public static final int FingersDuration = 20;
  public boolean fingersActive = false;
  public long fingersElapsed;
  public double fingersPower;

  @Override
  public void robotInit() {

    try{
      xbox = new XboxController(0);
      testController = makeSpark(6);
      elevatorController = makeSpark(7);
      coneController = makeSpark(8);
      fingerController = makeSpark(9);

      mecanum = makeMecanum();

      try{
        encoder = new Encoder(8,9);
        encoder.reset();
      }
      catch (Exception ex)
      {
        log(String.format("Encoder on 8,9 failed with\r\n%s", ex.getMessage()));
        encoder = null;
      }
    }
    catch (Exception ex)
    {
      xbox = null;
      testController = null;
      elevatorController = null;
      coneController = null;
      fingerController = null;
      encoder = null;
      log(String.format("RobotInit failed with\r\n%s", ex.getMessage()));
    }
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    testControllerPeriodic();
    elevatorPeriodic();
    conePeriodic();
    fingersPeriodic();
    encoderPeriodic();
    positionElevatorPeriodic();
    mecanumPeriodic();
  }


  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public static void log(String s){
		System.out.println(new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.S").format(new Date()) + "  " + s);
    }

  public Spark makeSpark(int channel){
    Spark spark;
    try{
      spark = new Spark(channel);
      spark.set(0);
    }
    catch (Exception ex)
    {
      log(String.format("Spark on %d failed with\r\n%s", channel, ex.getMessage()));
      spark = null;
    }
    return spark;
  }



  public void testControllerPeriodic(){
    if (testController == null)
      return;
  
    double power = xbox.getX(Hand.kRight);
    power = power * Math.abs(power);
    testController.set(power);
}


  public void elevatorPeriodic(){
    if (elevatorController == null)
      return;

    double power = xbox.getTriggerAxis(Hand.kRight) - xbox.getTriggerAxis(Hand.kLeft);
    power *= Math.abs(power);
    elevatorController.set(power);
  }

  public void conePeriodic(){
    if (coneController == null)
      return;

    double power = 0;
    if (xbox.getXButton())
      power = -ConePowerDefault;
    else if (xbox.getYButton())
      power = ConePowerDefault;
    coneController.set(power);
  }

  public void fingersPeriodic(){
    if (fingerController == null)
      return;

    boolean aPressed = xbox.getAButtonPressed();
    boolean bPressed = xbox.getBButtonPressed();
    if (aPressed || bPressed){
      if (aPressed)
        log("A Pressed");
      if (bPressed)
        log("B Pressed");
      fingersActive = true;
      fingersElapsed = 0;
      fingersPower = FingersPowerDefault;
      if (bPressed)
        fingersPower *= -1;
    }
    
    if (fingersActive){
      if (fingersElapsed <= FingersDuration)
        fingersElapsed++;
      else{
        fingersActive = false;
        fingersElapsed = 0;
        fingersPower = 0;
      }
    }

    fingerController.set(fingersPower);
  }

  public void encoderPeriodic(){
    if (encoder == null)
      return;

    SmartDashboard.putNumber("Raw", encoder.getRaw());
    SmartDashboard.putNumber("Distance", encoder.getDistance());
    SmartDashboard.putBoolean("Direction", encoder.getDirection());
    SmartDashboard.putData("Encoder", encoder);
  }
  public  void positionElevatorPeriodic(){
    if (elevatorController == null || encoder == null || !xbox.getBumper(Hand.kRight))
      return;
    
      double target = 10000;
      double range = 1000;
      double deadband = 50;
      double rMin = target - deadband - range;
      double dMin = target - deadband;
      double dMax = target + deadband;
      double rMax = target + deadband + range;

      double maxPower = 0.50;
      double minPower = 0.10;
      
      double x = encoder.getDistance();

      double power = 0;
      if (x < rMin)
        power = maxPower;
      else if (x <= dMin)
        power = minPower + (maxPower-minPower) * (dMin-x)/range;
      else if (x <= dMax)
        power = 0;
      else if (x <= rMax)
        power = -(minPower + (maxPower - minPower) * (dMax-x)/range);
      else if (x > rMax)
        power = -maxPower;
      elevatorController.set(power);
  }

  public MecanumDrive makeMecanum(){
    try{
      rearLeft = new WPI_TalonSRX(0);
      rearLeft.set(0);
      frontLeft = new WPI_TalonSRX(1);
      frontLeft.set(0);
      frontRight = new WPI_TalonSRX(2);
      frontRight.set(0);
      rearRight = new WPI_TalonSRX(3);
      rearRight.set(0);
      mecanum = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
      mecanum.driveCartesian(0, 0, 0);
    }
    catch (Exception ex){
      log(String.format("MecanumDrive failed with\r\n%s", ex.getMessage()));
      mecanum = null;
    }
    return mecanum;
  }

  public void mecanumPeriodic(){
    if (mecanum == null)
      return;
    
    double yPower =  xbox.getX(Hand.kLeft);
    double xPower = -xbox.getY(Hand.kLeft);
    double zPower =  xbox.getX(Hand.kRight);

    yPower *= Math.abs(yPower);
    xPower *= Math.abs(xPower);
    zPower *= Math.abs(zPower);

    mecanum.driveCartesian(yPower, xPower, zPower);

  }  
}
