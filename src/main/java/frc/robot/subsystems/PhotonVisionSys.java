package frc.robot.subsystems;
import org.photonvision.*;
public class PhotonVisionSys {
    public final PhotonCamera PhotonCam=new PhotonCamera("Aimbot9000");;
    public final double pitchOffSet= -34.372;
    public final double yawOffSet= 2.814;


    public boolean hasTargets(){
        var result = PhotonCam.getLatestResult();
        return result.hasTargets();
    }

    public double getArea(){
        var result = PhotonCam.getLatestResult();
        if(result.hasTargets()){
            return result.getBestTarget().getArea();
        }
        return 0;
    }

    private double getPitchOffset(){
        return (-5.02513)*getArea()+pitchOffSet;
    }

    private double getYawOffset(){
        return (-2.51256)*getArea()+yawOffSet;
    }

    public double getPitch(){
        var result = PhotonCam.getLatestResult();
        if(result.hasTargets()){
           return result.getBestTarget().getPitch()+getPitchOffset();
        }
        return 0;
    }

    public double getYaw(){
        var result = PhotonCam.getLatestResult();
        if(result.hasTargets()){
            return result.getBestTarget().getYaw()+getYawOffset();
        }
        return 0;
    }
}
