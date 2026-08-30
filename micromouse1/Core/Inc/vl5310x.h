#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "main.h"
#include <stdbool.h>

//Dia chi I2C dang 80bit cho 4 cam bien Laser
#define VL53L0X_ADDR_DEFAULT        (0x29 << 1)
#define VL53L0X_ADDR_LEFT			(0x30 << 1) // Ngang trái(PA3)
#define VL53L0X_ADDR_DIAG_LEFT		(0x32 << 1) //Cheo trai (PA4)
#define VL53L0X_ADDR_DIAG_RIGHT		(0X34 << 1) //Cheo phai(PB0)
#define VL53L0X_ADDR_RIGHT 					(0x36 << 1) //Ngang phai (PB1)

//Nguong phat hien tuong (mm) - Chinh lai tuy theo kich thuoc o sa ban
#define THRESHOLD_SIDE_MM 		      120 //Khoang cach nhan biet tuong cua cam bien ngang
#define THRESHOLD_DIAG_FRONT_MM     160 // Khoang cach cam bien cheo nhan biet tuong TRUOC


typedef struct
{
	uint16_t left; 				// chieu ngang 90 do trai
	uint16_t diag_left;  	// Chieu cheo 45 do truoc - trai
	uint16_t diag_right; 	// Chieu cheo 45 do truoc - phai
	uint16_t right; 				// Chieu ngang 90 phai
}

Distance_Data_t; // ==> luu khoang cach tu 4 huong xuong quanh
extern Distance_Data_t sensor_dist; //khai bao bien sensor_dist da duoc tao o mot file nguon .c

void VL53L0X_Init_All(void); //khoi tao cau hinh cho cac cam bien khong nhan tham so khong tra gia tri
void VL53L0X_Read_All(Distance_Data_t *dist); // ham doc du lieu khoang cach tu tat ca cac cam bien nhaan con tro truoc *

// API kiem tra vat can cho thuat toan Flood Fill
bool wallFront(void);
bool wallLeft(void);
bool wallRight(void);



#endif /* __VL53L0X_H */

