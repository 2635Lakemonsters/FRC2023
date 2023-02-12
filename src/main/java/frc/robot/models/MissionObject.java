package frc.robot.models;

// is this right?!?
import edu.wpi.first.math.geometry.Rotation2d;

public class MissionObject {
    // need field specific constants noting offset from the target april tag
    // that is being referenced to get base distance from the robot center to the april tag
    // to each of the positions so the results can be passed to an autonomous command.
    // maybe don't need constants at the top, since this is a MissionObject specific
    // to this year's curent game, could hard-code those into the functions since 
    // they will never be really re-used
    

    // want to return translation and rotation
    // likely want x, y, and yaw.
    // do we have a 
    public Rotation2d getFieldDeltaXYToScoringPositionLeft(){
        return null;
    }
    public Rotation2d getFieldDeltaXYToScoringPositionCenter(){
        return null;
    }
    public Rotation2d getFieldDeltaXYToScoringPositionRight(){
        return null;
    }

    // get better names for Position 1,2,3
    public Rotation2d getFieldDeltaXYToPickupPosition1(){
        return null;
    }
    public Rotation2d getFieldDeltaXYToPickupPosition2(){
        return null;
    }
    public Rotation2d getFieldDeltaXYToPickupPosition3(){
        return null;
    }
    
}


