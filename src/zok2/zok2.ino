#include "SPI.h"
#include "Mirf.h"
#include "nRF24L01.h"
#include "MirfHardwareSpiDriver.h"

#define PulsePin 0
#define vib_motor1 7
#define vib_motor2 8
#define vib_motor3 12
#define vib_motor4 13
#define peltier 10
#define pattern_num 10
#define vib_thr 10

volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 0;                     // used to find peak in pulse wave, seeded
volatile int T = 1000;                     // used to find trough in pulse wave, seeded
volatile int thresh = 525;                // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

volatile int BPM, preBPM;                  // int that holds raw Analog in 0. updated every 2mS
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile int Signal;                // holds the incoming raw data pattern;


void vib(int pattern) {
	int j;
	switch (pattern) {
		case 0: //全て同時
			analogWrite( vib_motor1, 255 );
			analogWrite( vib_motor2, 255 );
			analogWrite( vib_motor3, 255 );
			analogWrite( vib_motor4, 255 );
			delay(1000);
			analogWrite( vib_motor1, 0 );
			analogWrite( vib_motor2, 0 );
			analogWrite( vib_motor3, 0 );
			analogWrite( vib_motor4, 0 );
			break;
		case 1: //1,2->3,4
			analogWrite( vib_motor1, 255 );
			analogWrite( vib_motor2, 255 );
			delay(500);
			analogWrite( vib_motor1, 0 );
			analogWrite( vib_motor2, 0 );

			analogWrite( vib_motor3, 255 );
			analogWrite( vib_motor4, 255 );
			delay(500);
			analogWrite( vib_motor3, 0 );
			analogWrite( vib_motor4, 0 );
			break;
		case 2: //1,4->2,3
			analogWrite( vib_motor1, 255 );
			analogWrite( vib_motor4, 255 );
			delay(500);
			analogWrite( vib_motor1, 0 );
			analogWrite( vib_motor4, 0 );

			analogWrite( vib_motor2, 255 );
			analogWrite( vib_motor3, 255 );
			delay(500);
			analogWrite( vib_motor2, 0 );
			analogWrite( vib_motor3, 0 );
			delay(1000);
			break;
		case 3: //2,3->1,4
			analogWrite( vib_motor2, 255 );
			analogWrite( vib_motor3, 255 );
			delay(500);
			analogWrite( vib_motor2, 0 );
			analogWrite( vib_motor3, 0 );

			analogWrite( vib_motor1, 255 );
			analogWrite( vib_motor4, 255 );
			delay(500);
			analogWrite( vib_motor1, 0 );
			analogWrite( vib_motor4, 0 );
			delay(1000);
			break;
		case 4: //全て2回
			for (j = 0; j < 2; j++) {
				analogWrite( vib_motor1, 255 );
				analogWrite( vib_motor2, 255 );
				analogWrite( vib_motor3, 255 );
				analogWrite( vib_motor4, 255 );
				delay(475);
				analogWrite( vib_motor1, 0 );
				analogWrite( vib_motor2, 0 );
				analogWrite( vib_motor3, 0 );
				analogWrite( vib_motor4, 0 );
				if (i < 1) {
					delay(50);
				}
			}
			break;
		case 5: //全て3回
			for (j = 0; j < 3; j++) {
				analogWrite( vib_motor1, 255 );
				analogWrite( vib_motor2, 255 );
				analogWrite( vib_motor3, 255 );
				analogWrite( vib_motor4, 255 );
				delay(300);
				analogWrite( vib_motor1, 0 );
				analogWrite( vib_motor2, 0 );
				analogWrite( vib_motor3, 0 );
				analogWrite( vib_motor4, 0 );
				if (i < 2) {
					delay(50);
				}
			}
			break;
		case 6: //徐々に上がる
			for (j = 0; j < 256; j++) {
				analogWrite( vib_motor1, j );
				analogWrite( vib_motor2, j );
				analogWrite( vib_motor3, j );
				analogWrite( vib_motor4, j );
				delay((3000 / 256));
			}
			analogWrite( vib_motor1, 0 );
			analogWrite( vib_motor2, 0 );
			analogWrite( vib_motor3, 0 );
			analogWrite( vib_motor4, 0 );
			break;
		case 7: //徐々に下がる
			for (j = 255; j > -1; j--) {
				analogWrite( vib_motor1, j );
				analogWrite( vib_motor2, j );
				analogWrite( vib_motor3, j );
				analogWrite( vib_motor4, j );
				delay((3000 / 256));
			}
			break;
		case 8: //上がって下がる
			for (j = 0; j < 256; j++) {
				analogWrite( vib_motor1, j );
				analogWrite( vib_motor2, j );
				analogWrite( vib_motor3, j );
				analogWrite( vib_motor4, j );
				delay((1500 / 256));
			}
			for (j = 255; j > -1; j--) {
				analogWrite( vib_motor1, j );
				analogWrite( vib_motor2, j );
				analogWrite( vib_motor3, j );
				analogWrite( vib_motor4, j );
				delay((1500 / 256));
			}
			break;
		case 9: //下がって上がる
			for (j = 255; j > -1; j--) {
				analogWrite( vib_motor1, j );
				analogWrite( vib_motor2, j );
				analogWrite( vib_motor3, j );
				analogWrite( vib_motor4, j );
				delay((1500 / 256));
			}
			for (j = 0; j < 256; j++) {
				analogWrite( vib_motor1, j );
				analogWrite( vib_motor2, j );
				analogWrite( vib_motor3, j );
				analogWrite( vib_motor4, j );
				delay((1500 / 256));
			}
			analogWrite( vib_motor1, 0 );
			analogWrite( vib_motor2, 0 );
			analogWrite( vib_motor3, 0 );
			analogWrite( vib_motor4, 0 );
			break;
	}
}

void setup() {
	// put your setup code here, to run once:
	Serial.begin(9600);
	Mirf.spi = &MirfHardwareSpi;
	Mirf.init();
	Mirf.setRADDR((byte *)"device1");
	Mirf.payload = sizeof(unsigned long);
	Mirf.config();

	pinMode(peltier, OUTPUT);
	randomSeed(analogRead(0));
	digitalWrite(peltier, LOW);
}

void loop() {
	// put your main code here, to run repeatedly:
	Signal = analogRead(PulsePin);              // read the Pulse Sensor ｃ
	sampleCounter = millis();                         // keep track of the time in mS with this variable
	int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise


	//  find the peak and trough of the pulse wave
	if (Signal < thresh && N > (IBI / 5) * 3) { // avoid dichrotic noise by waiting 3/5 of last IBI
		if (Signal < T) {                       // T is the trough
			T = Signal;                         // keep track of lowest point in pulse wave
		}
	}

	if (Signal > thresh && Signal > P) {        // thresh condition helps avoid noise
		P = Signal;                             // P is the peak
	}                                        // keep track of highest point in pulse wave

	//  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	// signal surges up in value every time there is a pulse
	if (N > 250) {                                  // avoid high frequency noise
		if ( (Signal > thresh) && (Pulse == false) && (N > (IBI / 5) * 3) ) {
			Pulse = true;                               // set the Pulse flag when we think there is a pulse
			IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
			lastBeatTime = sampleCounter;               // keep track of time for next pulse

			if (secondBeat) {                      // if this is the second beat, if secondBeat == TRUE
				secondBeat = false;                  // clear secondBeat flag
				for (int i = 0; i <= 9; i++) {       // seed the running total to get a realisitic BPM at startup
					rate[i] = IBI;
				}
			}

			if (firstBeat) {                       // if it's the first time we found a beat, if firstBeat == TRUE
				firstBeat = false;                   // clear firstBeat flag
				secondBeat = true;                   // set the second beat flag
				sei();                               // enable interrupts again
				return;                              // IBI value is unreliable so discard it
			}


			// keep a running total of the last 10 IBI values
			word runningTotal = 0;                  // clear the runningTotal variable

			for (int i = 0; i <= 8; i++) {          // shift data in the rate array
				rate[i] = rate[i + 1];                // and drop the oldest IBI value
				runningTotal += rate[i];              // add up the 9 oldest IBI values
			}

			rate[9] = IBI;                          // add the latest IBI to the rate array
			runningTotal += rate[9];                // add the latest IBI to runningTotal
			runningTotal /= 10;                     // average the last 10 IBI values
			BPM = 60000 / runningTotal;             // how many beats can fit into a minute? that's BPM!
			Serial.print(BPM);
			if ((BPM - preBPM) > vib_thr) {
				digitalWrite(peltier, HIGH);
				long pattern = random(300);
				vib((pattern % pattern_num));
				Serial.print(" ");
				Serial.print((pattern % pattern_num));
				delay(2000);
				digitalWrite(peltier, LOW);
			}
			Serial.println();
			preBPM = BPM;
		}
		// QS FLAG IS NOT CLEARED INSIDE THIS ISR
	}

	if (Signal < thresh && Pulse == true) {  // when the values are going down, the beat is over
		Pulse = false;                         // reset the Pulse flag so we can do it again
		amp = P - T;                           // get amplitude of the pulse wave
		thresh = amp / 2 + T;                  // set thresh at 50% of the amplitude
		P = thresh;                            // reset these for next time
		T = thresh;
	}

	if (N > 2500) {                          // if 2.5 seconds go by without a beat
		thresh = 512;                          // set thresh default
		P = 0;                               // set P default
		T = 1000;                               // set T default
		lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
		firstBeat = true;                      // set these to avoid noise
		secondBeat = false;                    // when we get the heartbeat back
	}
}
