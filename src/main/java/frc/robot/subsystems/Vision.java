package frc.robot.subsystems;

// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Vision extends SubsystemBase {
  private static final int BAUD = 9600;
  private double angle;
  private double yPos;
  private SerialPort camPort;
  private double number;
  // private UsbCamera visionCam;

  private boolean isConnected = false;

  public Vision() {

    // visionCam = new UsbCamera("cam0", 1);
    // visionCam = CameraServer.startAutomaticCapture();
    // visionCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 30);

    try {
      System.out.println("1st Try");
      camPort = new SerialPort(BAUD, SerialPort.Port.kUSB);
    } catch (Exception e) {
      System.out.println("Error - 2nd Try");
      try {
        camPort = new SerialPort(BAUD, SerialPort.Port.kUSB1);
      } catch (Exception j) {
        try {
          System.out.println("Error - 3rd Try");
          camPort = new SerialPort(BAUD, SerialPort.Port.kUSB2);
        } catch (Exception k) {
          System.out.println("Could not connect robot to jevois");
        }
      }
    }
    if (camPort != null) {
      isConnected = true;
      packetListenerThread.start();
    }
    SmartDashboard.putNumber("angle", number);
    SmartDashboard.putBoolean("Vision Serial Port", isConnected);
  }

  // @Override
  public void backgroundUpdate() {
    try{
      String data = camPort.readString();
      String[] realData = data.split(";");
      if (realData.length == 2) {
        angle = Double.valueOf(realData[0]);
        yPos = Double.valueOf(realData[1]);
        System.out.println("VISION ANGLE: "+angle);
      }
      
    } catch (Exception e) {
      isConnected = false;
      packetListenerThread.interrupt();
      System.out.println("I AM MAKING IS CONNECTED FALSE");
      SmartDashboard.putBoolean("Vision Serial Port", isConnected);
      camPort.close();
    } 
    Timer.delay(0.05);
  }

  public double getAngle() {
    // return SmartDashboard.getNumber("angle", 10);
    return angle;
  }

  public double getInches(){
    System.out.println("IT IS THIS MANY INCHES AWAY: "+(8.92-3) / Math.tan((22.5+(yPos/240*45))*Math.PI/180));
    return (8.92-2.625) / Math.tan((37+(yPos/240*45))*Math.PI/180);
  }

  public double getFrontRPM() {
    return 215685/yPos - 4550;
  }

  public double getBackRPM() {
    return -107843/yPos + 2275;
  }

  Thread packetListenerThread = new Thread(new Runnable() {
    public void run() {
      while (!packetListenerThread.isInterrupted() && isConnected) {
        backgroundUpdate();
      }
      SmartDashboard.putBoolean("Vision Serial Port", false);
      System.out.println("AHHHHHHHHHHHH VISION IS DEAD MEAT U DID SOMETHING WRONG CUZ THE THINGY MABOBER DIED");
    }
  });
}
