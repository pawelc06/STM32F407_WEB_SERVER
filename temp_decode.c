#include "temp_decode.h"

char *my_strncpy(char *d, const char *s,unsigned int n)
{
   char *saved = d;
   unsigned char i=0;

   while (*s && ((i++)<n))
   {
       *d++ = *s++;
   }
   *d = 0;
   return saved;
}

unsigned char my_strlen(const char *s)
{

   unsigned char i;

   while (*s)
   {
       i++;
       s++;
   }

   return i;
}

unsigned char calculate_crc(char * buffer){
	unsigned char crc=0;
	int i = 0;

	while(buffer[i]){
		crc = crc ^ buffer[i];
		i++;
	}

	return crc;
}

unsigned char splitData(char *data,char *temp,char *voltage){

		my_strncpy(temp,data,5);
		temp[5] = 0;
		my_strncpy(voltage,data+6,5);



	//voltage[4] = 0;
	//voltage[5] = 'V';


	return 0;
}

unsigned char handleTempData(char *data,char *temp,char * vbat){
	unsigned char crc_calc;
	unsigned char crc_recv;



		int slen = my_strlen(data);

		if((data[0]!='+' && data[0]!='-')) //12 chars of data  plus crc
			return 0;




			 splitData(data,temp,vbat);





			 return 1;

}




