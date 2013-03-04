#ifndef DEF_GAURD_DO_CSR_H_
#define DEF_GAURD_DO_CSR_H_

/* Macros to SET do_csr register */
/*
  NOTE

  reset function by definition will give the following 
  command --
 
 */ 
#define RESET_OCSR(x)       x =  0x00000601

#define WIDTH_NOT32_OCSR(x) x &= 0xfffffffe
#define WIDTH_32_OCSR(x)    x |= 0x00000001

#define CLOCK_TIMER_OCSR(x) x &= 0xfffffff9
#define CLOCK_20MHZ_OCSR(x) x =  (x & 0xfffffff9) | 0x00000002
#define CLOCK_10MHZ_OCSR(x) x =  (x & 0xfffffff9) | 0x00000004
#define CLOCK_SHAKE_OCSR(x) x =  (x & 0xfffffff9) | 0x00000006

#define WAIT_NAE_OCSR(x)    x |= 0x00000008
#define NO_WAIT_NAE_OCSR(x) x &= 0xfffffff7

#define PAT_GEN_OCSR(x)     x |= 0x00000010
#define NO_PAT_GEN_OCSR(x)  x &= 0xffffffef

#define TRIGGER_OCSR(x)     x |= 0x00000020
#define NO_TRIG_OCSR(x)     x &= 0xffffffdf

#define TERM_ON_OCSR(x)     x &= 0xffffffbf
#define TERM_OFF_OCSR(x)    x |= 0x00000040

#define TRIGGER_END_OCSR(x) x |= 0x00000080
#define NO_TRIG_END_OCSR(x) x &= 0xffffff7f

#define ENABLE_OCSR(x)      x |= 0x00000100
#define DISABLE_OCSR(x)     x &= 0xfffffeff

#define CLEAR_FIFO_OCSR(x)  x |= 0x00000200
#define SAVE_FIFO_OCSR(x)   x &= 0xfffffdff

#define CLEAR_UNDER_OCSR(x) x |= 0x00000400

#define HANDSHAKE_OCSR(x)   x |= 0x00002000
#define NO_SHAKE_OCSR(x)    x &= 0xffffdfff

/* macros to CHECK DO CSR register */

#define FIFO_FULL_OCSR(x)   x & 0x00000800 
#define FIFO_EMPTY_OCSR(x)  x & 0x00001000
#define FIFO_UNDER_OCSR(x)  x & 0x00000400

#endif
