package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Scoring;

public class ScoringCommand extends Command{

    Constants.ScoringPos scoringPos;
    Scoring score;
    Timer timer;

    boolean finished;
    public ScoringCommand(Scoring score, Constants.ScoringPos scoringPos) {
        this.scoringPos = scoringPos;
        this.score = score;
        addRequirements(score);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
        finished = false;
        score.setArmFFEnable(false);

        if (scoringPos.equals(Constants.ScoringPos.GROUND) || scoringPos.equals(Constants.ScoringPos.STORE) && score.getArmAngle() < Constants.Arm.tempArmPosition) {
            score.goToPosition(Constants.ScoringPos.TEMP);
        }
        
    }

    @Override
    public void execute() {
        if (score.isFinished() || timer.get() > 2) {
            score.goToPosition(scoringPos);
            if (scoringPos.equals(Constants.ScoringPos.STORE)) {
                score.setArmFFEnable(true);
            }

            finished = true;
        } 
        
        
    }
    

    @Override
    public boolean isFinished() {
        return finished;
    }

}