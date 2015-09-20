#include <xbee-arduino\XBee.h>
#include <Wire\Wire.h>
//#include <SoftwareSerial\SoftwareSerial.h>

/*
This sketch is getting the data from NAU7802 using the I2C protocol and send the data to the coordinator Xbee.
It listens to the coordinator for instruction for the following instructions:
1. "o" opening the ball valve
2. "c" closing the ball valve
3. "p" openning the ball valve and waiting until there is no change in the weight of the bucket and then closing the valve
4. "s" sending a ZB_TX_REQUEST to the attached Xbee with the adc data to be sent to the coordinator
5. "n" a number between 1-6 that corresponds to the color of the led (the weight is calculated my the computer connected to the coordinator, this weight is translated to the color map. both the computer and arduino know what color each number corresponds to.
6. "v" checks the state of the valve and sends it to xbee (1 = open, 0 = close)

*/

// create the XBee object
XBee xbee = XBee();

// this functions will setup the parameters in NAU7802 and initializes it
void NAU7802_setup();
// this is the function that reads the data from the NAU7802
uint16_t adc_read();
// this function opens the valve
void open_valve();
// this function closes the valve
void close_valve();
//this function opens the valve, waits until the bucket is empty and then closes the valve
void open_empty_close();
//this fuction sets the color of the led based on the color value that is passes on to it
//void set_color(uint8_t R, uint8_t G, uint8_t B, uint16_t delaymili);
//void set_color(uint8_t color);
//this function sends the load cell data to the coordinator
void send_data();
//void send_simulated_data();
//this function checks the state of the valve and sends it to the xbee
void send_valve_state();

//global loadcell data 
uint16_t adc = 0;
uint8_t adc_mid = 0;
uint8_t adc_high = 0;

//outputs from Arduino to activate the ball valve
//digital pin 9 is the out1 and digital pin 10 is the out2
uint8_t valveOut1 = 9; //connects to IN1 on L298N board
uint8_t valveOut2 = 10; //connects to IN2 on L298N board
//analog pins A0 and A1 are reserved to check the position of the ball valve (close or open)
//if pin A0 goes low, the valve is open and if pin A1 goes low, the valve is closed
uint8_t valveOpen = A0;
uint8_t valveClose = A1;
//digital pins 11, 12 and 13 are to control the RGB led light
uint8_t redPin =5;
uint8_t greenPin = 6;
uint8_t bluePin = 3; 

//this is the minimum weight that we can read from the NAU7802 
uint16_t minWeight = 100;


uint8_t data[] = {0x48, 0x49};
uint16_t adcVec[20];
//number of readings from the loadcell
uint8_t numread = 20;
//number of outliers to throw away from each end
uint8_t outlier = 5;

//union u_tag {
//    uint8_t b[2];
//    int integer;
//}u;

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x00000000, 0x00000000);
//instantiating the Zigbee transmit request object
ZBTxRequest zbTx = ZBTxRequest(addr64, data, sizeof(data));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

//create reusable response objects for responses we expect to handle
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

//SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {

    Wire.begin();
    Serial.begin(9600);
    //mySerial.begin(9600);
    xbee.setSerial(Serial);

    

    pinMode(valveOut1, OUTPUT);
    pinMode(valveOut2, OUTPUT);

    pinMode(valveOpen, INPUT);
    pinMode(valveClose, INPUT);
    digitalWrite(valveClose, HIGH);
    digitalWrite(valveOpen, HIGH);

    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);

	NAU7802_setup();//-------------------------------------------------------------
    //set_color(10, 0, 0, 50);
    //set_color(0, 10, 0, 50);
    //set_color(0, 0, 10, 50);

}

void loop() {
	
    //set_color(1, 0, 1, 50);
     //wait for an instruction from the coordinator
    xbee.readPacket();

    if (xbee.getResponse().isAvailable())
    {
        //got something
        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE)
        {
            //got a zb rx packet
            //now fill our zb rx class
            xbee.getResponse().getZBRxResponse(rx);

            //set_color(0, 10, 0, 20); //flash green to show that we received a ZB_RX_RESPONSE packet
            //set_color(0, 10, 0, 20);

            if (rx.getOption() == ZB_PACKET_ACKNOWLEDGED)
            {
                // the sender got and ACK
            }
            else
            {
                // we got it but the sender didn't get an ACK
            }
            //uint8_t length = rx.getDataLength();
            //here it is a good idea to report the response somehow for debugging purposes

            //here we read the first byte of the packet and run instructions based on it
            uint8_t data = rx.getData()[0];
            if (data == 'o'){ open_valve(); }
            else if (data == 'c'){ close_valve(); }
            else if (data == 'p'){ /*open_empty_close();*/ }
            else if (data == 'v'){ send_valve_state(); }
            else if (data == 's'){ send_data();/* send_simulated_data();/* send_data();*/ }
            else if (data == 'n')
            { /*
                set_color(int(rx.getData()[1] - '0')); 
                delay(1000); */
            }
            else{/* none of the options apeared to be in the first byte, this must be an error*/ }
        }
        else if (xbee.getResponse().getApiId()==MODEM_STATUS_RESPONSE)
        {
            xbee.getResponse().getModemStatusResponse(msr);
            //the local XBee sends this response on certain events, like association/dissociation

            if (msr.getStatus() == ASSOCIATED)
            {
                //yay this is great!
                //set_color(0, 20, 0, 100);
                //set_color(0, 20, 0, 100);
            }
            else if (msr.getStatus()==DISASSOCIATED)
            {
                //this is not good
                //set_color(20, 0, 0, 100);
                //set_color(20, 0, 0, 100);
            }
            else
            {
                //another status
            }
        }
        else
        {
            //received a packet but it is not a zb_rx_response !
        }
    }
    else if (xbee.getResponse().isError())
    {
        //set_color(250, 0, 250, 100);
        //set_color(250, 0, 250, 100);
        //Error Reading packet
        // the error code can be retrived using xbee.getResponse().getErrorCode();
    }

}

void NAU7802_setup()
{
    //set_color(0, 0, 255, 100); //flash blue once to show the start of the calibration

    Wire.beginTransmission(0x2A);     // Reset all the registers by writing 1 to the RR bit of REG00(0)
	Wire.write(0x00);
	Wire.write(0b00000001);
	Wire.endTransmission();

	Wire.beginTransmission(0x2A);     // Turn the device on and set the RR to 0
	Wire.write(0x00);
	Wire.write(0b10000110);
	Wire.endTransmission();

	//  Wire.beginTransmission(0x2A);     // Enable strong pullup for the I2C bus
	//  Wire.write(0x11);
	//  Wire.write(0b00100000);
	//  Wire.endTransmission();  

	Wire.beginTransmission(0x2A);     // Selece the x128 gain by writing 111 to GAINS bit of REG01(2:0) 
	Wire.write(0x01);                 // Select the 4.5V for VLDO by writing 000 to VLDO bits of REG01(5:3)
	Wire.write(0b10000111);
	Wire.endTransmission();

	Wire.beginTransmission(0x2A); // Initiate the internal calibration by writing 1 to bit CALS of REG02(2)
	Wire.write(0x02);
	Wire.write(0b00100100);
	Wire.endTransmission();

	delay(1500); //wait for the calibration to finish

	Wire.beginTransmission(0x2A);     // transmit to device 
	Wire.write(0x1B);
	Wire.write(0b00100001);
	Wire.endTransmission();
	////  
	Wire.beginTransmission(0x2A);     // Turn off the ADC choppers
	Wire.write(0x15);
	Wire.write(0b00110000);
	Wire.endTransmission();
	//
	Wire.beginTransmission(0x2A);     // Enables PGA output bypass capacitor connected across pins Vin2P Vin2N
	Wire.write(0x1C);
	Wire.write(0b10000000);
	Wire.endTransmission();
	//  
	//  //the CAL_ERR register in the REG02(3) should be zero for the calibration to be error free
	//Serial.println("If REG02 is not equal to zero, the calibration didn't work!");
	//  
	//Read all the registers and print them on Serial
    /*
    Wire.beginTransmission(0x2A);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(0x2A, 0x20);
	int i = 0;
	while (Wire.available())
	{
		byte c = Wire.read();          // receive a byte as character
		Serial.print(i, HEX);
		Serial.print(" = ");
		i++;
		Serial.println(c, BIN);             // print the character
	}
	//  Wire.endTransmission();

	Serial.println("End - ADC");
    */
	delay(1000);

    //set_color(0, 0, 255, 100); //flash blue twice to show the end of the calibration
    //set_color(0, 0, 255, 100);
}

uint16_t adc_read()
{
	Wire.beginTransmission(0x2A);
	Wire.write(0x12);
	Wire.endTransmission();
	Wire.requestFrom(0x2A, 1);
	while (Wire.available())
	{
		adc_high = Wire.read();
	}
	Wire.beginTransmission(0x2A);
	Wire.write(0x13);
	Wire.endTransmission();
	Wire.requestFrom(0x2A, 1);
	while (Wire.available())
	{
		adc_mid = Wire.read();
	}
	//Wire.beginTransmission(0x2A);
	//Wire.write(0x14);
	//Wire.endTransmission();
	//Wire.requestFrom(0x2A, 1);
	//int c = 0;
	//while (Wire.available()) {
	//	c = Wire.read();
	//}
	//uint16_t adc = 0;
	adc = (adc_high << 8) + adc_mid;
	return (adc);
}

// this function opens the valve
void open_valve()
{
	digitalWrite(valveOut1, HIGH);
	digitalWrite(valveOut2, LOW);
	while (digitalRead(valveOpen))
	{
		//wait while valveOpen pin is HIGH which mean the valve is not fully open
        //set_color(0, 0, 5, 100);
        delay(100);
	}
	digitalWrite(valveOut1, LOW);
	digitalWrite(valveOut2, LOW);
}

// this function closes the valve
void close_valve()
{
	digitalWrite(valveOut1, LOW);
	digitalWrite(valveOut2, HIGH);
	//valveClose and valveOpen pins will go LOW if the ball valve is open and close respectively
	while (digitalRead(valveClose))
	{
		//wait while valveClose pin is HIGH which mean the valve is not fully closed
        //set_color(5, 0, 0, 100);
        delay(100);
	}
	digitalWrite(valveOut1, LOW);
	digitalWrite(valveOut2, LOW);
}

//this function checks the state of the valve and sends it to xbee
void send_valve_state()
{
    uint8_t valve_state = 2;
    if (digitalRead(valveClose)) valve_state = 0;
    if (digitalRead(valveOpen)) valve_state = 1;
    data[1] = 0;
    data[0] = valve_state;
    xbee.send(zbTx);

    // flash TX indicator
    //set_color(200, 0, 200, 100);
}

//this function opens the valve, waits until the bucket is empty and then closes the valve
//void open_empty_close()
//{
//	//open the valve-----------------------------------
//	digitalWrite(valveOut1, HIGH);
//	digitalWrite(valveOut2, LOW);
//	while (valveOpen)
//	{
//		//wait while valveOpen pin is HIGH which mean the valve is not fully open
//	}
//	digitalWrite(valveOut1, LOW);
//	digitalWrite(valveOut2, LOW);
//	//-------------------------------------------------
//	
//	//read the data from load cell (NAU7802) and stay open until all the water is emptied----
//	//--------------------------------------------------
//	int weight = adc_read();
//	int oldWeight = 0;
//	while (abs(weight - oldWeight) > minWeight)
//	{
//		//weight until the new reading is within the minWeight of the old reading
//		delay(1000);
//		oldWeight = weight;
//		weight = adc_read();
//	}
//	digitalWrite(valveOut1, LOW);
//	digitalWrite(valveOut2, HIGH);
//	//---------------------------------------------------
//	
//	//close the valve------------------------------------
//	while (valveClose)
//	{
//		//wait while valveClose pin is HIGH which mean the valve is not fully closed
//	}
//	digitalWrite(valveOut1, LOW);
//	digitalWrite(valveOut2, LOW);
//	//--------------------------------------------------
//}

//this fuction sets the color of the led based on the color value that is passes on to it
//if delaymili is set to zero, it sets the color and exits, 
//if delaymili is anyother number, it turns the led on for delaymili miliseconds and then turns it off
//void set_color(uint8_t R, uint8_t G, uint8_t B, uint16_t delaymili)
//{
//    if (delaymili == 0)
//    {
//        analogWrite(redPin, R);
//        analogWrite(greenPin, G);
//        analogWrite(bluePin, B);
//    }
//    else
//    {
//        analogWrite(redPin, R);
//        analogWrite(greenPin, G);
//        analogWrite(bluePin, B);
//        delay(delaymili);
//        analogWrite(redPin, 0);
//        analogWrite(greenPin, 0);
//        analogWrite(bluePin, 0);
//        delay(delaymili);
//    }
//}

//void set_color(int color)
//{
//    /*
//    0 - red
//    1 - yellow
//    2 - light green
//    3 - dark green
//    4 - blue
//    */
//    switch (color)
//    {
//    case 0:
//        set_color(10,0,0,0);
//        break;
//    case 1:
//        set_color(10, 10, 0, 0);
//        break;
//    case 2:
//        set_color(0, 10, 10, 0);
//        break;
//    case 3:
//        set_color(5, 5, 5, 0);
//        break;
//    case 4:
//        set_color(10,0,10,0);
//        break;
//    default:
//        set_color(0,0,5,0);
//    }
//}

//this function will send the load cell data to the coordinator
void send_data()
{
    for (uint8_t i = 0; i < numread; i++)
    {
        adc_read();
        adcVec[i] = (adc_high << 8) + adc_mid;
    }

	for (int x = 0; x<numread; x++)
	{
		bool done = true;
		for (int y = 0; y<numread - 1; y++)
		{
			if (adcVec[y]>adcVec[y + 1])
			{
				done = false;
				int temp = adcVec[y + 1];
				adcVec[y + 1] = adcVec[y];
				adcVec[y] = temp;
			}
		}
		if (done) break;
	}

    uint16_t sum;
	for (uint8_t i = outlier; i < numread - outlier; i++)
    {
        sum += adcVec[i];
    }
    
    sum /= (numread-2.0*outlier);

    data[1] = sum >> 8;
    data[0] = sum & 0b11111111;
    xbee.send(zbTx);

    // flash TX indicator
    //set_color(200, 0, 200, 100);

    // after sending a tx request, we expect a status response
    // wait up to half second for the status response
      if (xbee.readPacket(500)) {
        // got a response!
    
        // should be a znet tx status            	
        if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
          xbee.getResponse().getZBTxStatusResponse(txStatus);
    
          // get the delivery status, the fifth byte
          if (txStatus.getDeliveryStatus() == SUCCESS) {
            // success.  time to celebrate
          } else {
            // the remote XBee did not receive our packet. is it powered on?
          }
        }
      } else if (xbee.getResponse().isError()) {
        //nss.print("Error reading packet.  Error code: ");  
        //nss.println(xbee.getResponse().getErrorCode());
      } else {
        // local XBee did not provide a timely TX Status Response -- should not happen
      }
}

//void send_simulated_data()
//{
//    data[0] = int(random(0, 255));
//    data[1] = int(random(0, 255));
//    xbee.send(zbTx);
//
//    // flash TX indicator
//    //set_color(200, 0, 200, 100);
//
//    // after sending a tx request, we expect a status response
//    // wait up to half second for the status response
//    /*
//    if (xbee.readPacket(500)) {
//        // got a response!
//
//        // should be a znet tx status            	
//        if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
//            xbee.getResponse().getZBTxStatusResponse(txStatus);
//
//            // get the delivery status, the fifth byte
//            if (txStatus.getDeliveryStatus() == SUCCESS) {
//                // success.  time to celebrate
//            }
//            else {
//                // the remote XBee did not receive our packet. is it powered on?
//            }
//        }
//    }
//    else if (xbee.getResponse().isError()) {
//        //nss.print("Error reading packet.  Error code: ");  
//        //nss.println(xbee.getResponse().getErrorCode());
//    }
//    else {
//        // local XBee did not provide a timely TX Status Response -- should not happen
//    }
//    */
//}
