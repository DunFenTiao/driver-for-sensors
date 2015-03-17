# driver-for-sensors
sensors:
- iaq:co2,vco2
- tmg3992:proximity,ALS
- mpu6050:x,y,z,ax,ay,az

platform:
- iaq:arduino;Intel Edison;stm32;
- tmg3992:arduino;Intel Edison;stm32;
- mpu6050:Intel Edison;



