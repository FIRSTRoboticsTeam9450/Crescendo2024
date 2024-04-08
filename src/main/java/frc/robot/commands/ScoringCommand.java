package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Scoring;

public class ScoringCommand extends Command{

    Constants.ScoringPos scoringPos;
    Scoring score;


    public ScoringCommand(Scoring score, Constants.ScoringPos scoringPos) {
        this.scoringPos = scoringPos;
        this.score = score;
        addRequirements(score);
    }

    @Override
    public void initialize() {
        score.goToPosition(scoringPos);
    }
    

    @Override
    public boolean isFinished() {
        return true;
    }

}