//////////////////////////////////////////////////////////////////		Deklaracija Varijabli		///////////////////////////////////////////////////////////////////

byte ldr_1 = A1;		//pinovi senzora za pracenje crte
byte ldr_2 = A2;		
int ldr1, ldr2;			//varijable s kojima ocitavamo te senzore

byte trig = 3;			//pin za slanje impulsa ultrazvucnim senzorom 
byte echo = 2;			//pin za primanje impulsa na ultrazvucnom senzoru

byte obs_dist = 40;		//zeljena udaljenost na kojoj robot staje prije nekog predmeta na stazi
int dist;			//trenutna udaljenost do najblizeg predmeta isperd robota

byte button = 7;		//gumb za podesavanje kalibracije senzora
byte led = 6;			//ledica za komunikaciju s korisnikom

int black1, black2;		//varijable za vrijedonsti koje dobijemo za vijeme kalibracije
int white1, white2;
int grey1, grey2;		//zavrsene filtrirane varijable vrijedosti za pracenje crte

byte a1 = 9, a2 = 8;		//pinovi za motor a(naprijed i nazad)
byte b1 = 12, b2 = 13;		//i za motor b(isto naprijed i nazad)
byte pwma = 10, pwmb = 11;	//pin za podesanvanje tocne brzine motora

//////////////////////////////////////////////////////////////////		Pocetak programa		///////////////////////////////////////////////////////////////////

void setup()
{
	Serial.begin(9600);	//pocetak seriske komunikacije (racunalo <--> robot)
	Serial.println("Start :) :) :)");
				//mala sala za pocetak programam [Albert ili Danek ko god da je pls nemojte ovo maknut]

	delay(200);		//malo pricekati

	//inputs
	pinMode(ldr_1, INPUT);	//deklaracija ulaza na robota
	pinMode(ldr_2, INPUT);
	pinMode(button, INPUT_PULLUP);
	
	pinMode(trig, OUTPUT);	//dekalracija pinova ultrazvucnog senzora
	pinMode(echo, INPUT);

	//outputs
	pinMode(a1, OUTPUT);	//deklaracija izlaznih uredaja robota
	pinMode(a2, OUTPUT);
	pinMode(b1, OUTPUT);
	pinMode(b2, OUTPUT);
	pinMode(pwma, OUTPUT);
	pinMode(pwmb, OUTPUT);
	pinMode(led, OUTPUT);
	pinMode(led_ldr, OUTPUT);

	Serial.println("Calibration ...... ");
		
	//test_motors();	//ako prvi puta koristimo robota mozemo provijetiti rade li motori propisno

	while(digitalRead(button) == 1){}		//cekamo da se stisne gumb
	

				//zapocinjemo kalibraciju za vrijednosti crne trake
	for(int i = 0; i<16; i++)	
	{
		black1 += analogRead(ldr_1);		//16 puta u varijable zbrojimo trenutne vrijedosti
		black2 += analogRead(ldr_2);
		digitalWrite(led, HIGH);
		delay(15);
		digitalWrite(led, LOW);
		delay(15);
	}

	black1 = black1 / 16;				//te ovdje ih podijeliom s 16 da bismo dobili aritmeticku sredinu
	black2 = black2 / 16;

	while(digitalRead(button) == 1){}		//cekamo da se gubm opet stisne
	
				//ista kalibracija sao za bijelu boju
	for(int i = 0; i<16; i++)
	{
		white1 += analogRead(ldr_1);
		white2 += analogRead(ldr_2);
		digitalWrite(led, HIGH);
		delay(15);
		digitalWrite(led, LOW);
		delay(15);
	}

	white1 = white1 / 16;
	white2 = white2 / 16;

	grey1 = (black1+white1)/2;			//ove vrijedoti ce nam treabti kasnije za pracenje crte a zapravo 
	grey2 = (black2+white2)/2;			//a zapravo je samo aritmeticka sredina vrijdnosti crne i bijele za svaki senzor posebno

	Serial.println("Valuse r:");			//mozemo ako hocemo ispisati preko serijske komunikacije na racunalu vrijesoti za svaki parametar
	Serial.print(white1);
	Serial.print("	");
	Serial.print(white2);
	Serial.print("    ##    ");
	Serial.print(black1);
	Serial.print("	");
	Serial.println(black2);

	delay(300);

	Serial.print(grey1);
	Serial.print("	");
	Serial.println(grey2);
	
	delay(300);

	while(digitalRead(button) == 1)			//za pocetak programa cekamo da se stisne gumb
	{}

	delay(500);

	digitalWrite(led, HIGH);			//upalimo i ugasuimo jednu ledicu da bismo korisniku rekli da ce sada robot
	delay(50);					//poceti pratitit crtu
	digitalWrite(led, LOW);
	delay(50);
}

///////////////////////////////////////////////////////////////		Pocetak glavnog dijela programa		/////////////////////////////////////////////////////////////////

void loop()
{
	ldr1 = analogRead(ldr_1);			//prvo da bismo pratili crtu ili znali udaljenosto od neke prepreke moramo ocitati senzore prvo 
	ldr2 = analogRead(ldr_2);			//one za crtu a onda 
	readUltra();					//ultrazvucni

	if(digitalRead(button) == 0)			//ako se tijekom pracenja crte pritisne nas gumb onda robot stane i jednom led diodom pokaze
	{						//korisniku da je stao tako sto ju upali
		while(1)
		{
			digitalWrite(led, HIGH);
			motors(0, 0);
		}
	}

	Serial.print(ldr1);				//ako zelimo mozemo na racunalo ispisivati vrijdnosti senzora
	Serial.print("	");
	Serial.print(ldr2);
	Serial.print("	##	");
	Serial.println(dist);				
	
	if(dist < obs_dist)				//provijeravamo da li je udaljenosti izmedu obijekta ispred robota manja od ono zadane za stop
	{
		while(dist < obs_dist)			//ako je onda dok se taj obijek ne maken
		{
			motors(0, 0);			//robot stoji
			readUltra();			//te provijerava da li se maknuo
		}
	}

//////////////////////////////////////////////////////////////		Pracenje crte 				///////////////////////////////////////////////////////////////////

	if(ldr1 < grey1)				//provijeravamo da li je lijevi senzor na crnom ili bijelom	
	{
		if(ldr2 < grey2)			//ako je na bijelom onda provijeravamo dali je drugi(desni) senzor na crnom ili bijelom
		{
			motors(255, 255);		//ako su oba na bijelom onda kazemo robotu da ide ravno
		}
		else
		{
			motors(255, 0);			//a ako je drugi na crnom a prvi na bijelom onda se okrecemo prema desno da bismo se ispravili
		}
	}
	else						//ako je prvi na crnom
	{
		if(ldr2 < grey2)			//a drugi na bijelom
		{
			motors(0, 255);			//onda se okrecemo premao prvom(lijevo) da bismo se ispravlili
		}
		else					//a ako su oba na crnom
		{
			motors(255, 255);		//onda samo idemo ravno
		}
	}

	delay(10);					//na kraju malo pricekamo kako bi motor imao vremena reagirati i onda se opet ponavlja pracenje crte i provijera
							//za ultrazvucni senzor	
}

////////////////////////////////////////////////////////////		Motori					///////////////////////////////////////////////////////////////////

void motors(int m1, int m2)				//ovo je prodprocedura(subrutina, ili podprogram) koja sluzi za upravljanje motora
{							//kada ju pozivamo damo joj dva parametra m1 i m2 sto su brzine i smjerovi za prvi i drugi motor
	if(m1 < 0)					//ako je m1 < 0 onda motor 1 palimo u natrah
	{
		//back
		digitalWrite(a1, HIGH);
		digitalWrite(a2, LOW);
	}
	else if(m1 == 0)				//ako je m1 == 0 onda gasimo taj motor
	{
		//stop
		digitalWrite(a1, LOW);
		digitalWrite(a2, LOW);
	}
	else						//inace(m1 > 0) palimo motor u naprijed
	{
		//forward
		digitalWrite(a1, LOW);
		digitalWrite(a2, HIGH);
	}

	if(m2 < 0)					//isto je za i m2 ako je manji od nule onda je u natrag
	{
		//back
		digitalWrite(b1, HIGH);
		digitalWrite(b2, LOW);
	}
	else if(m2 == 0)				//ako je 0 onda stani
	{
		//stop
		digitalWrite(b1, LOW);
		digitalWrite(b2, LOW);
	}
	else						//a inace idi naprijed
	{
		//forward
		digitalWrite(b1, LOW);
		digitalWrite(b2, HIGH);
	}

	//pwm speed thingy :)

	analogWrite(pwma, abs(m1));			//s obzirom da smo podesili smijerove nasih motora sad trebamo postaviti brzinu brzina varira od -255
	analogWrite(pwmb, abs(m2));			//do 255, sto su krajnje vrijedosti tj. max brzina u naprijed ili nazad
							//ali ako hocemo da nas robot ide u naprijde u pola brzine onda bismo napisali motors(128, 128);
}

////////////////////////////////////////////////////////////		Ocitanje Ultrazvucnih senzora			///////////////////////////////////////////////////////////

void readUltra()					//u ovoj podproceduri ocitavamo ultrazvucni senzor
{
	digitalWrite(trig, HIGH);			//on radi tako da upali neki pin kako bi posalo neki puls
	delayMicroseconds(10);
	digitalWrite(trig, LOW);			//onda ga ugasi 
	delayMicroseconds(5);

	int us = pulseIn(echo, HIGH, 10000);		//te ceka da vidi koliko ce mu trebati da se vrati
	dist = us/2/29;					//onda to preracuna u cm

	if(dist == 0)					//i jos smo dodali da ako je obijekt predaleko tj senzor ga ne ocitiava da posavi udaljenost na 1023
	{
		dist = 1023;
	}
	
	Serial.println(dist);				//i jos na kraju na racunalu ispisemo vrijednost
}

///////////////////////////////////////////////////////////7		Testiranje motora				///////////////////////////////////////////////////////////

void test_motors()					//ovo je potprocedura u kojoj mozemo testirati naseg robota
{
	motors(255, 255);				//prvo ide naprijed max brzina
	delay(1000);

	motors(255, -255);				//onda skrece lijevo-desno ali tako da jedan motor ide naprijed a drugi nazad
	delay(1000);
	motors(-255, 255);
	delay(1000);

	motors(255, 0);					//nakon toga skrece na dugi nacin skrece lijevo-desno ali tako da jedan motor ide ravno a drugi stoji
	delay(1000);
	motors(0, 255);
	delay(1000);

	motors(-255, -255);				//onda se varti unazad
	delay(1000);

	motors(0, 0);					//te ugasi motore
							//uz malo srece on se vartio na pocetnu poziciuju kretanja
}
