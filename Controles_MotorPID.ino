int encoder_pin1 = 2;  // The pin the encoder is connected
int encoder_pin2 = 3;  // The pin the encoder is connected
double rpm1;     // rpm motor 1
double rpm2;     // rpm motor 2
volatile byte pulses1;  // number de pulsos 1
volatile byte pulses2;  // number de pulsos 2
unsigned long timeold1;
unsigned long timeold2;
// The numero de pulsos por revolucion
double pulsesperturn = 20;

int motor1=9; //Declara Pin del motor Derecho
int motor2=5; //Declara Pin del motor Izquierdo

int pinVCCEncoder1 = 12;
int pinVCCEncoder2 = 13;
int band = 0;

double unidades = 50; //decisegundos

void counter1()
{
    //Update count
    pulses1++;
}

void counter2()
{
    //Update count
    pulses2++;
}

/** Tiempo de Muestreo **/
double T = 0.050;

/** LEFT PID MOTOR VARIABLES **/
double izq_Kp = 208.29;
double izq_Ki = 2941.18;
double izq_Kd = 2.74;

double izq_E0 = 0;
double izq_E1 = 0;

double izq_Dk = 0;
double izq_Ik = 0;
double izq_Ik1 = 0;

double izq_Uk = 0;
/** END  **/

/** RIGHT PID MOTOR VARIABLES **/
double der_Kp = 125.654;
double der_Ki = 2094.24;
double der_Kd = 1.8325;

double der_E0 = 0;
double der_E1 = 0;

double der_Dk = 0;
double der_Ik = 0;
double der_Ik1 = 0;

double der_Uk = 0;
/** END  **/

/** REFERENCES **/
/** Values between 0 to 20 (o to 1 revolutions per time unit) **/
double refIzq = 13; //Left Reference
double refDer = 13; //Right Reference

void setup()
{

    Serial.begin(9600); //Serial Comunincation with PC

    pinMode(motor1, OUTPUT); //Set output pins for motors
    pinMode(motor2, OUTPUT);

    pinMode(pinVCCEncoder1, OUTPUT);
    pinMode(pinVCCEncoder2, OUTPUT);
    //Use statusPin to flash along with interrupts
    pinMode(encoder_pin1, INPUT);
    pinMode(encoder_pin2, INPUT);

    //Interrupt 0 is digital pin 2, so that is where the IR detector is connected
    //Triggers on FALLING (change from HIGH to LOW)
    attachInterrupt(digitalPinToInterrupt(encoder_pin2), counter1, FALLING);
    attachInterrupt(digitalPinToInterrupt(encoder_pin1), counter2, FALLING);
    // Initialize
    pulses1 = 0;
    rpm1 = 0;
    timeold1 = 0;
    pulses2 = 0;
    rpm2 = 0;
    timeold2 = 0;
    band = 0;
    digitalWrite(pinVCCEncoder1, HIGH);
    digitalWrite(pinVCCEncoder2, HIGH);

    analogWrite(motor1, 170);
    analogWrite(motor2, 200);
}

void loop()
{

    if (millis() - timeold1 >= unidades)   /*Uptade every fifty miliseconds, this will be equal to reading frecuency (Hz).*/
    {
        detachInterrupt(digitalPinToInterrupt(encoder_pin1));
        //rpm1 =  (double)(millis() - timeold1)* (pulses1/pulsesperturn) / (unidades*10);
        rpm1 = pulses1;
        timeold1 = millis();
        band += 1;
        //Write it out to serial port
        Serial.print("1,");
        Serial.println(rpm1, DEC);
        pulses1 = 0;

        /** Calculo Accion del PID **/
        izq_E0 = refIzq - rpm1;
        //Serial.println(izq_E0, DEC);
        izq_Dk = (izq_Kd/T)*(izq_E0 - izq_E1);
        izq_Ik = izq_Ik1 + izq_Ki*izq_E0;
        izq_Uk = izq_Kp*izq_E0 + izq_Dk + izq_Ik;

        if(izq_Uk > 200 || izq_Uk < 0){
          izq_Ik = izq_Ki*izq_E0;
        }
        if(izq_Uk > 200){
          izq_Uk = 200;
        }
        if(izq_Uk < 0){
          izq_Uk =80;
        }
        analogWrite(motor1, izq_Uk); //Control Actions
        izq_Ik1 = izq_Ik;
        izq_E1 = izq_E0;

        
        
        //Restart the interrupt processing
        attachInterrupt(digitalPinToInterrupt(encoder_pin1), counter1, FALLING);

        
    
    }

    if (millis() - timeold2 >= unidades)   /*Uptade every fifty miliseconds, this will be equal to reading frecuency (Hz).*/
    {
        detachInterrupt(digitalPinToInterrupt(encoder_pin2));
        //rpm2 =  (double)(millis() - timeold2)* (pulses2/pulsesperturn) / (unidades*10);
        rpm2 = pulses2;
        timeold2 = millis();
        //Write it out to serial port
        Serial.print("2,");
        Serial.println(rpm2, DEC);
        //Restart the interrupt processing

        /** Calculo Accion del PID **/
        der_E0 = refDer - rpm1;
        //Serial.println(der_E0, DEC);
        der_Dk = (der_Kd/T)*(der_E0 - der_E1);
        der_Ik = der_Ik1 + der_Ki*der_E0;
        der_Uk = der_Kp*der_E0 + der_Dk + der_Ik;
        if(der_Uk > 200 || der_Uk < 0){
          der_Ik = der_Ki*der_E0;
        }
        if(der_Uk > 200){
          der_Uk = 200;
        }
        if(der_Uk < 0){
          der_Uk = 80;
        }
        analogWrite(motor2, der_Uk); //Control Actions
        der_Ik1 = der_Ik;
        der_E1 = der_E0;
        
        attachInterrupt(digitalPinToInterrupt(encoder_pin2), counter2, FALLING);
        pulses2 = 0;
    }
}


