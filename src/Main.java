import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;


public class Main {
	//sensory
	public static EV3UltrasonicSensor DS = new EV3UltrasonicSensor(SensorPort.S4);
	public static EV3LargeRegulatedMotor MotorA = new EV3LargeRegulatedMotor(MotorPort.A);
	public static EV3LargeRegulatedMotor MotorD = new EV3LargeRegulatedMotor(MotorPort.D);
	static SensorModes S4 = DS;
	static SampleProvider S4Sample = S4.getMode(0);
	static float[] sampleS4Data = new float[S4Sample.sampleSize()];

	//nastavime si globalni promenne
	public static double time = 0;
	public static double dist = 0;
	public static long startTime = System.currentTimeMillis();
	public static int initialDistLeft = MotorA.getTachoCount();
	public static int initialDistRight = MotorD.getTachoCount();
	public static double r = 56;
	public static double wallDist;
	public static int cubeNum = 0;
	public static int wallNum = 0;
	public static void main(String[] args) {
		//predpokladejme, ze robot zacina pred modrou krychli

		//nastavime si local main promenne
		int fastSpeed = 200;
		int slowSpeed = 50;

		double slowZone = 50; //delka zony zpomanleni pred a za kostkou

		/*	dva sety vzdalenosti a delky
		 * 	vybereme si pouze jeden a ten ulozime do promenne keepDist/walllengt
		 * 	hodnoty v metrech
		 *  hodnoty walldist pouzivame kdyz udrzujeme vzdalenost robota od steny
		 *  hodnoty wallLenght pouzivame kdyz potrebujeme delku steny
		 * */
		//double wallDistA[] = {1400, 2800, 1400, 2800};
		//double wallDistB[] = {1400, 2800, 1400, 2800};

		//double keepWallDist[] = wallDistA;

		double wallLenghtA[] = {1260, 1120, 1680, 1120};
		//double wallLenghtB[] = {1260, 1120, 1680, 1120};

		double wallLenght[] = wallLenghtA;
		/*
		 * Set vzdáleností krychlí vždy v dané vzdálenosti je krychle
		 * pøed krychlí robot zpomalí a pøipraví se na zvedání krychle
		 * v metrech jsou krychle za sebou {0, 0.28, 0.56, 0.84} v m
		 * */
		double cubeDist[] = {0, 280, 560, 840};
		double curretntCubeDist;

		resetDist();
		setSpeed(fastSpeed);
		go();

		System.out.println("Distance Traveled: " + dist + " mm");
		System.out.println("wall lenght: " + wallLenght[wallNum] + " mm");

		// Main loop
		while (!Button.ESCAPE.isDown()) {

			//update hodnot time a dist
			update();

			/*        	System.out.println("Time Elapsed: " + time + " ms");
            System.out.println("Distance Traveled: " + dist + " mm");
            System.out.println("Nearest wall: " + wallDist + " mm");*/

			//pøepínání rychlosti podle vzdálenosti od kostky
			curretntCubeDist = cubeDist[cubeNum] - (dist + slowZone/2.0);
            if(Math.abs(curretntCubeDist) < slowZone/2)
            {
            	System.out.println("--------------- CUBE ZONE ------------------");
            	if (MotorA.getSpeed() != slowSpeed) {
                	//	robot vstoupil do kostkové zóny
                	System.out.println("--------------- CUBE ZONE ENTERED ------------------");
            		setSpeed(slowSpeed);
                	go();
                	Sound.beep();
                	//cubeLift();
				}
            }else if(MotorA.getSpeed() != fastSpeed)
            {
            	setSpeed(fastSpeed);
            	go();
            }

            System.out.println("Distance Traveled: " + dist + " mm");

			//pokud je robot na konci steny
			if(wallLenght[wallNum] - dist < 1)
			{
				System.out.println("--------------TURNING LEFT----------------");
				System.out.println("Distance Traveled before turning: " + dist + " mm");
				turnLeft();
				System.out.println("Distance Traveled after turning: " + dist + " mm");
				resetDist();
				setSpeed(fastSpeed);
				go();

			}
			Delay.msDelay(100);
		}

		stop();
	}

	public static void cubeLift()
	{

		// zajistuje zvedání krychle
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

		// vyýpoèet ujeté vzdálenosti
		int currentTachoCountLeft = MotorA.getTachoCount();
		int currentTachoCountRight = MotorD.getTachoCount();
		int distanceDgLeft = currentTachoCountLeft - initialDistLeft;
		int distanceDgRight = currentTachoCountRight - initialDistRight;
		double distDg = (distanceDgLeft + distanceDgRight) / 2.0; // prùmìrná ujetá vzdálenost ve stupních

		dist = degToMm(distDg); // prumnerna ujeta vzdalenost v mm

		wallDist = getDistanceValue();
	}

	public static double degToMm(double degrees)
	{
		return  Math.PI/360 * r * degrees;
	}

	public static double mmToDeg(double mm)
	{
		return  (360/(Math.PI * r)) * mm;
	}

	public static void resetDist()
	{
		//nastaví prvotní ujetou vzdálenost na aktulání - reset ujeté vzdálenosti
		initialDistLeft = MotorA.getTachoCount();
		initialDistRight = MotorD.getTachoCount();
	}

	public static void go()
	{
		MotorA.forward();
		MotorD.forward();
	}

	public static void stop()
	{
		MotorA.stop();
		MotorD.stop();
	}

	public static void setSpeed(int velocity)
	{
		stop();
		MotorA.setSpeed(velocity);
		MotorD.setSpeed(velocity);
	}

	public static double turnR = 140; // polomer zatacky mm
	public static void turnLeft()
	{
		int initSpeedA = MotorA.getSpeed();
		int initSpeedD = MotorD.getSpeed();

		double disA = ((turnR - 85) * Math.PI * 2)/4;
		double disB = ((turnR + 85) * Math.PI * 2)/4;

		stop();
		MotorD.setSpeed((int) Math.abs(disA/5));
		MotorA.setSpeed((int) Math.abs(disB/5));
		MotorD.rotate((int) mmToDeg(disA), true);
		MotorA.rotate((int) mmToDeg(disB));
		cubeNum = 0;
		wallNum++;
		stop();
		MotorA.setSpeed(initSpeedA);
		MotorD.setSpeed(initSpeedD);
	}
	public static int getDistanceValue()
	{
		S4Sample.fetchSample(sampleS4Data, 0);
		return (int) (1000 * sampleS4Data[0]);
	}
}

//TODO
/*
 */