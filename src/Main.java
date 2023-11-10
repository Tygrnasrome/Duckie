import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;


public class Main {
	//nastavime si globalni promenne
	public static double time = 0;
	public static double dist = 0;
	public static long startTime = System.currentTimeMillis();
	public static int initialDistLeft = Motor.A.getTachoCount();
	public static int initialDistRight = Motor.D.getTachoCount();
	public static double r = 0.056;
	public static double wallDist;
	public static int cubeNum = 0;
	public static int wallNum = 0;
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

	public static void update()
	{
		//update hodnot time, dist a wallDist

        long currentTime = System.currentTimeMillis();
        time = currentTime - startTime;

        // vyýpoèet ujeté vzdálenosti
        int currentTachoCountLeft = Motor.A.getTachoCount();
        int currentTachoCountRight = Motor.D.getTachoCount();
        int distanceDgLeft = currentTachoCountLeft - initialDistLeft;
        int distanceDgRight = currentTachoCountRight - initialDistRight;
        double distDg = (distanceDgLeft + distanceDgRight) / 2.0; // prùmìrná ujetá vzdálenost ve stupních
        dist = Math.PI/180 * r * distDg; // prumnerna ujeta vzdalenost v metrech

        wallDist = getDistanceValue();
	}

	public static void resetDist()
	{
		//nastaví prvotní ujetou vzdálenost na aktulání - reset ujeté vzdálenosti
		initialDistLeft = Motor.A.getTachoCount();
		initialDistRight = Motor.D.getTachoCount();
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