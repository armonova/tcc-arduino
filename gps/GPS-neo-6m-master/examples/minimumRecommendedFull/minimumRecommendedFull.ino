#include <SoftwareSerial.h>
#include <Gpsneo.h>

Gpsneo gps(4, 3, 9600);


char latitude[15];
char latitudHemisphere[2];
char longitude[15];
char longitudeMeridian[15];
char speedKnots[10];
void  setup()
{
    Serial.begin(9600);
    Serial.println("Opa 1");
}

void loop()
{
//  char x[2];
//  x[0]='z';
//  x[1]='\0';
//    Serial.println(/"Opa 2");
    //Just simple do getDataGPRMC, and the method solve everything.
//    gps.getDataGPRMC(timeV,statusV, latitud, latitudHemisphere, longitud,
//                     longitudMeridiano, speedKnots,trackAngle, date,
//                     magneticVariation, magneticVariationOrientation
//                     );
//    gps.getDataGPRMC(&x[0],&x[0],latitud,latitudHemisphere,longitud,longitudMeridiano, /&x[0],&x[0],&x[0],&x[0],&x[0]);

//    gps.getDataGPRMC(&x[0],&x[0],latitude,latitudHemisphere,longitude,longitudeMeridian, speedKnots,&x[0],&x[0],&x[0],&x[0]);
    gps.getDataGPRMC(latitude,latitudHemisphere,longitude,longitudeMeridian);

//    Serial.print/ln("Opa 3");

//     Serial.println(timeV);/
//     Serial.println(statusV);
     Serial.println(latitude);
     Serial.println(latitudHemisphere);
     Serial.println(longitude);
     Serial.println(longitudeMeridian);
     Serial.println(speedKnots);
//     Serial.println(trackAngle);
//     Serial.println(date);
//     Serial.println(magneticVariation);
//     Serial.println(magneticVariationOrientation);
}
