import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;


public class Main {
	//motory
	public static EV3LargeRegulatedMotor MotorA = new EV3LargeRegulatedMotor(MotorPort.A);
	public static EV3LargeRegulatedMotor MotorD = new EV3LargeRegulatedMotor(MotorPort.D);
	//sensory
	public static EV3UltrasonicSensor DS = new EV3UltrasonicSensor(SensorPort.S4);
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
	public static double wheelDist = 85;
	public static double wallDist;
	public static int cubeNum = 0;
	public static int wallNum = 0;
	public static int currentSpeed;

	public static double keepWallDist[];
	public static double turnR[]; // polomer zatacky mm

	public static void main(String[] args) {
		//predpokladejme, ze robot zacina pred modrou krychli

		//nastavime si local main promenne
		int fastSpeed = 200;
		int slowSpeed = 100;

		double slowZone = 25; //delka zony zpomanleni pred a za kostkou

		/*	dva sety vzdalenosti a delky
		 * 	vybereme si pouze jeden a ten ulozime do promenne keepDist/walllengt
		 * 	hodnoty v metrech
		 *  hodnoty walldist pouzivame kdyz udrzujeme vzdalenost robota od steny
		 *  hodnoty wallLenght pouzivame kdyz potrebujeme delku steny
		 * */
		double wallDistA[] = {67, 228, 67, 207};
		double wallDistB[] = {1400, 2800, 1400, 2800};

		keepWallDist = wallDistA;

		double wallLenghtA[] = {1260, 840, 1680, 1120};
		//double wallLenghtB[] = {1260, 1120, 1680, 1120};

		double wallLenght[] = wallLenghtA;
		/*
		 * Set vzdáleností krychlí vždy v dané vzdálenosti je krychle
		 * pøed krychlí robot zpomalí a pøipraví se na zvedání krychle
		 * v metrech jsou krychle za sebou {0, 0.28, 0.56, 0.84} v m
		 * */
		double cubeDist[] = {0, 280, 560, 840, 100000000};
		double wallFirstCubeDisp[] = {0, 0, 280, 280};
		double curretntCubeDist;

		double turnRA[] = {280, 280, 280};
		turnR = turnRA;
		resetDist();
		setSpeed(fastSpeed);
		go();

		// Main loop
		while (!Button.ESCAPE.isDown() || wallNum == 3) {

			//update hodnot time a dist
			update();

			speedCorrection();

			//pøepínání rychlosti podle vzdálenosti od kostky
			curretntCubeDist = (cubeDist[cubeNum] + wallFirstCubeDisp[wallNum]) - (dist + slowZone/2.0);
			if(Math.abs(curretntCubeDist) < (slowZone))
			{
				if (currentSpeed != slowSpeed) {
					//	robot vstoupil do kostkové zóny
					System.out.println("--------------------------------- CUBE " + cubeNum + " ZONE ENTERED --------------------------------");
					setSpeed(slowSpeed);
					go();
					//cubeLift();
				}
				System.out.println("--------------- CUBE " + cubeNum + " ZONE ------------------");
			}else if(currentSpeed != fastSpeed)
			{
				System.out.println("--------------------------------- CUBE " + cubeNum + " ZONE LEFT --------------------------------");
				setSpeed(fastSpeed);
				go();
				cubeNum++;
			}

			speedCorrection();

			System.out.println("Dist: " + dist + " mm");
			System.out.println("wallDist: " + wallDist + " mm");
			//pokud je robot na konci steny
			if(wallLenght[wallNum] - dist < 1)
			{
				System.out.println("--------------------------------TURNING LEFT---------------------------------------");
				System.out.println("Distance Traveled before turning: " + dist + " mm");
				System.out.println("A: " + MotorA.getTachoCount() + " mm");
				System.out.println("D: " + MotorD.getTachoCount() + " mm");

				turnLeft();
				setSpeed(fastSpeed);
				speedCorrection();
				update();
				System.out.println("Distance Traveled after turning: " + dist + " mm");

				resetDist();
				go();
			}
			Delay.msDelay(100);
		}

		stop();
	}

	public static void speedCorrection()
	{
		double error = getDistanceValue() - keepWallDist[wallNum];
		double correction = -error * 0.5f;

		MotorA.setSpeed((int) (currentSpeed + correction));
		MotorD.setSpeed((int) (currentSpeed - correction));
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
		MotorA.stop(true);
		MotorD.stop();
	}

	public static void setSpeed(int velocity)
	{
		currentSpeed = velocity;
		MotorA.setSpeed(velocity);
		MotorD.setSpeed(velocity);
	}

	public static void turnLeft()
	{
		int robotR = 180;

		double disA = ((turnR[wallNum] - robotR/2.0) * Math.PI)/2;
		double disB = ((turnR[wallNum] + robotR/2.0) * Math.PI)/2;

		stop();
		MotorD.setSpeed((int) Math.abs(disA/3));
		MotorA.setSpeed((int) Math.abs(disB/3));
		MotorD.rotate((int) mmToDeg(disA), true);
		MotorA.rotate((int) mmToDeg(disB));
		cubeNum = 0;
		wallNum++;
		stop();
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