package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.AxisCamera;

import java.lang.Thread;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Camera extends SubsystemBase {
  private String axisName = "axis-camera.local";
  Thread m_axisCamThread;

  public Camera() {
    m_axisCamThread = new Thread(new Runnable() {
      public void run() {
        AxisCamera axisCam = CameraServer.addAxisCamera(axisName);
        axisCam.setResolution(640, 480);
      }
    });
    m_axisCamThread.start();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop

  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}