package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link edu.wpi.first.wpilibj2.command.button.CommandPS5Controller}
 * with analog trigger methods matching the Xbox controller API.
 */
public class BetterCommandPS5Controller extends edu.wpi.first.wpilibj2.command.button.CommandPS5Controller {

    public BetterCommandPS5Controller(int port) {
        super(port);
    }

    public Trigger L2(double threshold, EventLoop loop) {
        return axisGreaterThan(PS5Controller.Axis.kL2.value, threshold, loop);
    }

    public Trigger L2(double threshold) {
        return L2(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger R2(double threshold, EventLoop loop) {
        return axisGreaterThan(PS5Controller.Axis.kR2.value, threshold, loop);
    }

    public Trigger R2(double threshold) {
        return R2(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
}