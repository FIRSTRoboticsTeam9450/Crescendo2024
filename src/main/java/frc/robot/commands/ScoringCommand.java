package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Scoring;

public class ScoringCommand extends Command{

    Constants.ScoringPos scoringPos;
    Scoring score;

    boolean finished;
    public ScoringCommand(Scoring score, Constants.ScoringPos scoringPos) {
        this.scoringPos = scoringPos;
        this.score = score;
        addRequirements(score);
    }

    @Override
    public void initialize() {
        finished = false;
        if (scoringPos.equals(Constants.ScoringPos.GROUND) || scoringPos.equals(Constants.ScoringPos.STORE) && score.getArmAngle() < Constants.Arm.tempArmPosition) {
            score.goToPosition(Constants.ScoringPos.TEMP);
        }
        
    }

    @Override
    public void execute() {
        if (score.isFinished()) {
            score.goToPosition(scoringPos);
            finished = true;
        } 
        
    }
    

    @Override
    public boolean isFinished() {
        return finished;
    }

}