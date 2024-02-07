package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {



    //private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
      

        configureButtonBindings();
        //autoChooser = AutoBuilder.buildAutoChooser();
        //AutoBuilder.followPath(PathPlannerPath.fromPathFile("New Path")).schedule();
        
    }

    private void configureButtonBindings() {
        
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        return null;
    }
}