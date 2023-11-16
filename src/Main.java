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

	public static double cubeDist[] = {0, 280, 560, 840, 100000000};
	public static double wallFirstCubeDisp[] = {0, 0, 280, 280};
	public static double keepWallDist[];
	public static double turnR[]; // polomer zatacky mm

	public static int fastSpeed = 200;
	public static int slowSpeed = 100;

	public static double slowZone = 25; //delka zony zpomanleni pred a za kostkou

	public static void main(String[] args) {
		//predpokladejme, ze robot zacina pred modrou krychli

		//nastavime si local main promenne




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




		double turnRA[] = {280, 280, 280};
		turnR = turnRA;
		resetDist();
		setSpeed(fastSpeed);
		go();

		// Main loop
		while (!Button.ESCAPE.isDown() || wallNum == 3) {

			//update hodnot time a dist
			update();
			if(!isTurning)
			{
				speedLogic();
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
				}
			}else
			{
				turningLogic();
			}
			Delay.msDelay(10);
		}

		stop();
	}

	public static void speedLogic()
	{
		double curretntCubeDist;
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
	public static int robotR = 180;
	public static double initATacho;
	public static double initDTacho;
	public static boolean isTurning = false;
	public static double ATurnDist;
	public static double DTurnDist;
	public static double ATurnTacho;
	public static double DTurnTacho;
	public static void turnLeft()
	{
		initATacho = MotorA.getTachoCount();
		initDTacho = MotorD.getTachoCount();

		ATurnDist = ((turnR[wallNum] + robotR/2.0) * Math.PI)/2;
		DTurnDist = ((turnR[wallNum] - robotR/2.0) * Math.PI)/2;

		MotorD.setSpeed((int) Math.abs(DTurnDist/3));
		MotorA.setSpeed((int) Math.abs(ATurnDist/3));

		ATurnTacho = mmToDeg(ATurnDist);
		DTurnTacho = mmToDeg(DTurnDist);

		isTurning = true;
	}

	public static void turningLogic()
	{
		double currentTurnTachoA = MotorA.getTachoCount() - initATacho;
		double currentTurnTachoD = MotorD.getTachoCount() - initDTacho;
		System.out.println("CurrentATurnTacho: " + currentTurnTachoA);
		System.out.println("CurrentDTurnTacho: " + currentTurnTachoD);
		if(currentTurnTachoD >= DTurnTacho/* && currentTurnTachoD >= DTurnTacho*/)
		{
			isTurning = false;
			resetDist();
			wallNum++;
			cubeNum = 0;
		}
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