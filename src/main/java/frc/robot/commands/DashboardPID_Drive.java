/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DashboardPID_Drive extends CommandBase {
  /**
   * Creates a new DashboardPID_Drive.
   */
  private final Drive drive;
  private double dist;
  private final double margin;
  private double error;
  private double kP, kI, kD, kF;
  private double p, i,d,ff;
  private double lastf, lastp, lasti, lastd;

  public DashboardPID_Drive(final Drive drive, final double dist, final double margin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;

    kF = 0; 
    kP = 0; 
    kI = 0; 
    kD = 0;

    SmartDashboard.putNumber("Feed Fwd", lastf);
    SmartDashboard.putNumber("k_P", lastp);
    SmartDashboard.putNumber("k_I", lasti);
    SmartDashboard.putNumber("k_D", lastd);

    this.dist = dist;
    this.margin = margin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.resetEncoders();

    drive.setkP(this.kP);
    drive.setkI(this.kI);
    drive.setkD(this.kD);
    drive.setkF(this.kF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    p = SmartDashboard.getNumber("P Gain", 0);
    i = SmartDashboard.getNumber("I Gain", 0);
    d = SmartDashboard.getNumber("D Gain", 0);
    ff = SmartDashboard.getNumber("Feed Forward", 0);

    dist = SmartDashboard.getNumber("SetPoint", 0);

    if((p != kP)) {drive.setkP(p); kP = p;}
      SmartDashboard.putNumber("set p", p);
    if((i != kI)) {drive.setkI(i); kI = i;}
      SmartDashboard.putNumber("set i", i);
    if((d != kD)) {drive.setkD(d); kD = d;}
      SmartDashboard.putNumber("set pd", d);
    if((ff != kF)) {drive.setkF(ff); kF = ff;}
      SmartDashboard.putNumber("set f", ff);

    drive.setOutputRange(Constants.DriveConstants.MIN_OUTPUT, Constants.DriveConstants.MAX_OUTPUT);

    drive.setSetPoint(dist);
    SmartDashboard.putNumber("Spark setPoint", dist);

    error = Math.abs(dist - drive.getLeftEncoder());
    SmartDashboard.putNumber("Spark error", error);

    SmartDashboard.putNumber("Spark L encoder", drive.getLeftEncoder());
    SmartDashboard.putNumber("Spark R encoder", drive.getRightEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isDistMargin = error < margin;
    return isDistMargin;
  }
}
