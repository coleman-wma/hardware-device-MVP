///SendTextMessage()
///this function is to send a sms message
void SendTextMessage()
{
  mySerial.print("AT+CMGF=1\r");    //Because we want to send the SMS in text mode
  delay(100);
  // Al = 0868145904 - Jason 0868201442
  mySerial.println("AT + CMGS = \"+353868201442\"");//send sms message, be careful need to add a country code before the cellphone number
  delay(100);
  mySerial.print("Hello Jason - this is SITESPY. The Temperature where you are at the moment is ");//the content of the message
  mySerial.print(Temp);//the content of the message
  mySerial.print(" degrees C. The pressure in mb is ");//the content of the message
  mySerial.print(absPressure);//the content of the message
  mySerial.print(", and the heading is ");//the content of the message
  mySerial.print(finalHead);//the content of the message
    mySerial.print(" degrees.");//the content of the message
  delay(100);
  mySerial.println((char)26);//the ASCII code of the ctrl+z is 26
  delay(100);
  mySerial.println();
}
