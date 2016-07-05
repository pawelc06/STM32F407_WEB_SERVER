/*
 * temp_decode.h
 *
 *  Created on: 4 kwi 2016
 *      Author: Pawe³
 */

#ifndef INC_TEMP_DECODE_H_
#define INC_TEMP_DECODE_H_

unsigned char my_strlen(const char *s);
char *my_strncpy(char *d, const char *s,unsigned int n);


unsigned char calculate_crc(char * buffer);
unsigned char splitData(char *data,char *temp,char *voltage);
unsigned char handleTempData(char *data,char *temp,char * vbat);


#endif /* INC_TEMP_DECODE_H_ */
