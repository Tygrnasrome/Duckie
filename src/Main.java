import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;


public class Main {
	//sensory
	public static EV3UltrasonicSensor DS = new EV3UltrasonicSensor(SensorPort.S1);
	static SensorModes S1 = DS;
	static SampleProvider S1Sample = S1.getMode(0);
	static float[] sampleS1Data = new float[S1Sample.sampleSize()];

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
		 * Set vzd�lenost� krychl� v�dy v dan� vzd�lenosti je krychle
		 * p�ed krychl� robot zpomal� a p�iprav� se na zved�n� krychle
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

            //p�ep�n�n� rychlosti podle vzd�lenosti od kostky
            double curretntCubeDist = cubeDist[cubeNum] - (dist + slowZone/2.0);
            if(Math.abs(curretntCubeDist) < slowZone/2)
            {
            	//robot je pobl� kostky
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

	public static void cubeLift()
	{

		// zajistuje zved�n� krychle
		if(Motor.C.isStalled())
		{
			Motor.C.rotate(300);
			Motor.C.rotate(-300);
		}
	}

	public static void update()
	{
		//update hodnot time, dist a wallDist

        long currentTime = System.currentTimeMillis();
        time = currentTime - startTime;

        // vy�po�et ujet� vzd�lenosti
        int currentTachoCountLeft = Motor.A.getTachoCount();
        int currentTachoCountRight = Motor.D.getTachoCount();
        int distanceDgLeft = currentTachoCountLeft - initialDistLeft;
        int distanceDgRight = currentTachoCountRight - initialDistRight;
        double distDg = (distanceDgLeft + distanceDgRight) / 2.0; // pr�m�rn� ujet� vzd�lenost ve stupn�ch
        dist = Math.PI/180 * r * distDg; // prumnerna ujeta vzdalenost v metrech

        wallDist = getDistanceValue();
	}

	public static void resetDist()
	{
		//nastav� prvotn� ujetou vzd�lenost na aktul�n� - reset ujet� vzd�lenosti
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
	public static int getDistanceValue()
	{
		S1Sample.fetchSample(sampleS1Data, 0);
		return (int) (100 * sampleS1Data[0]);
	}
}

//TODO
/*
 * na zaklade vzdalenosti menit rychlosti
 * na zaklade vzdalenosti chytani kostek
 * na zaklade vzdalenosti otocka
 * trackovani casu a vzdalenosti
 */