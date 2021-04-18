@Autonomous(name="Iron Golem Autonomous", group="Mechanum Bot")
public class RingLaunchandParkAuto extends AutonomousOpMode {
    @Override
    public void runOpMode() throws InterruptedException { 
        super.runOpMode();

        //EXAMPLE AUTONOMOUS CODE

        //Move Robot forward 8 inches at 50% speed
        //manualMove(8,8,8,8,0.5);

        //Turn Robot 90 Degrees at 50% speed
        //turn(90,0.5);

        //Move 8 Inches at -18 Degrees at 50% speed
        //move(-18,8,0.5);
        
        
        move(0,67,0.5);
        shoot(0.56);
        move(0,13,0.5);
        sleep(3000);
    }
}