import lejos.hardware.motor.Motor;


public class Main {
	//nastavime si globalni promenne
	public static double time = 0;
	public static double dist = 0;

	public static void main(String[] args) {
		//predpokladejme, ze robot zacina pred modrou krychli

		//nastavime si local main promenne
		int fastSpeed = 200;
		int slowSpeed = 50;

		/*	dva sety vzdalenosti
		 * 	vybereme si pouze jeden a ten ulozime do promenne keepDist
		 * 	hodnoty pouzivame kdyz udrzujeme vzdalenost robota od steny
		 * */
		double distA[] = {0.14, 0.28, 0.14, 0.28};
		double distB[] = {0.14, 0.28, 0.14, 0.28};

		//keepDist obsahuje pouzivanou sadu vzdalenosti kostek od steny
		double keepDist[] = distA;


	}

	public static void update(int velocity, double dt)
	{
		//update hodnot time a dist
		// dt je delta time
		time += dt;
		dist += velocity/dt;
		//motor tacho by mel fungovat lepe


	}

	public static void go()
	{
		Motor.A.forward();
		Motor.D.forward();
	}

	public static void stop()
	{
		Motor.A.stop();
		Motor.D.stop();
	}

	public static void setSpeed(int velocity)
	{
		Motor.A.setSpeed(velocity);
		Motor.D.setSpeed(velocity);
	}

	private static int turnAngle = 120;
	public static void turnLeft()
	{
		Motor.A.rotate(turnAngle, true);
		Motor.D.rotate(-turnAngle);
		cubeNum = 0;
	}
	}
}

//TODO
/*
 * na zaklade vzdalenosti meneni rychlosti
 * na zaklade vzdalenosti chytani kostek
 * na zaklade vzdalenosti otocka
 * trackovani casu a vzdalenosti
 */