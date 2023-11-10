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

		double slowZone = 0.1; //delka zony zpomanleni pred a za kostkou
		double turnZone = 0.1;

		/*	dva sety vzdalenosti a delky
		 * 	vybereme si pouze jeden a ten ulozime do promenne keepDist/walllengt
		 * 	hodnoty v metrech
		 *  hodnoty walldist pouzivame kdyz udrzujeme vzdalenost robota od steny
		 *  hodnoty wallLenght pouzivame kdyz potrebujeme delku steny
		 * */
		double wallDistA[] = {0.14, 0.28, 0.14, 0.28};
		double wallDistB[] = {0.14, 0.28, 0.14, 0.28};

		double keepWallDist[] = wallDistA;

		double wallLenghtA[] = {1.68, 1.68, 2.52, 1.68};
		double wallLenghtB[] = {1.68, 1.68, 2.52, 1.68};

		double wallLenght[] = wallLenghtA;
		/*
		 * Set vzdáleností krychlí vždy v dané vzdálenosti je krychle
		 * pøed krychlí robot zpomalí a pøipraví se na zvedání krychle
		 * v metrech jsou krychle za sebou {0, 0.28, 0.56, 0.84}
		 * */
		double cubeDist[] = {0, 0.28, 0.56, 0.84};

		resetDist();
        go();

        // Main loop
        while (!Button.ESCAPE.isDown()) {

        	//update hodnot time a dist
        	update();

            System.out.println("Time Elapsed: " + time + " ms");
            System.out.println("Distance Traveled: " + dist + " m");
            System.out.println("Nearest wall: " + wallDist + " m");

            //pøepínání rychlosti podle vzdálenosti od kostky
            double curretntCubeDist = cubeDist[cubeNum] - (dist + slowZone/2.0);
            if(Math.abs(curretntCubeDist) < slowZone/2)
            {
            	//robot je poblíž kostky
            	//setSpeed(slowSpeed);
            	//cubeLift();
            }else
            	setSpeed(fastSpeed);

            //pokud je robot na konci steny
            if(wallLenght[wallNum] - dist < turnZone)
            {
            	setSpeed(slowSpeed);
            	turnLeft();
            }
            Delay.msDelay(100);
        }

        // Stop motors on exit
        stop();
	}


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