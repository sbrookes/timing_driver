#ifndef DEF_GUARD_8254_TIMER_H_
#define DEF_GUARD_8254_TIMER_H_

/*

  NOTE --
  
  The PCI7300 uses the 3 timers in the following ways

      Counter 0 -- Input clock
      Counter 1 -- Output Clock
      Counter 2 -- Interrupt generation

 */

/* timer programming masks */
#define COUNTER_0_8254(x)  x = (x & 0x3f) 
#define COUNTER_1_8254(x)  x = (x & 0x3f) | 0x40
#define COUNTER_2_8254(x)  x = (x & 0x3f) | 0x80

#define LSB_ONLY_8254(x)   x = (x & 0xcf) | 0x10
#define MSB_ONLY_8254(x)   x = (x & 0xcf) | 0x20
#define LSB_TO_MSB_8254(x) x = x | 0x30

#define MODE_0_8254(x)     x = (x & 0xf1)
#define MODE_1_8254(x)     x = (x & 0xf1) | 0x02
#define MODE_2_8254(x)     x = (x & 0xf1) | 0x04
#define MODE_3_8254(x)     x = (x & 0xf1) | 0x06
#define MODE_4_8254(x)     x = (x & 0xf1) | 0x08
#define MODE_5_8254(x)     x = (x & 0xf1) | 0x0a

#define BINARY_8254(x)     x = (x & 0xfe) 
#define BCD_8254(x)        x = (x | 0x01)

/* timer status check masks */
#define READ_BACK_8254(x) x = (x & 0x3f) | 0xc0

#define OUT_LATCH_8254(x) x = (x & 0xcf)


#endif
