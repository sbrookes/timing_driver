#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/types.h>
#include "../../include/do_csr.h"
#include "../../include/8254_timer.h"

#define SIXTEEN_K 16*1024

int main(void) {

  int DO_CSR, DO_FIFO, output, TIMER_CTRL, TIMER_1, PLX_9080;
  FILE *ofile;
  unsigned char *fifo, writec;
  unsigned int i, j, k;
  __u32 cmd;
  unsigned char timer;
  unsigned char clock = 0x06;

  /* DEBUGGING FILE */
  output = 1;

  i = SIXTEEN_K; /* write 32 bit chunks */

  fifo = (unsigned char *)malloc(i*sizeof(char));
  if (!fifo) {
    printf("ERROR MALLOCING\n");
    exit(2);
  }

  k = SIXTEEN_K / 4;
  while ( k-- ) {
    writec = ( k % 2 ? 0x00 : (k % 4 ? 0xfe : 0xff));  
    for ( j = 0; j < 4; j++ )
      fifo[--i] = writec;
  }
  
  /* print to file to check what the buffer is */
  if ( output ) {
    ofile = fopen("first_test_results.txt", "w");
    for ( i = 0; i < SIXTEEN_K; i++ ) {
      if (fifo[i] == 0x00)
	fprintf(ofile, "00");
      else if (fifo[i] == 0xff)
	fprintf(ofile, "ff");
      else 
	fprintf(ofile, "??");
    }
    fclose(ofile);
  }

  DO_FIFO = open("/dev/timing5", O_WRONLY);
  DO_CSR  = open("/dev/timing1", O_RDWR);
 
  PLX_9080 = open("/dev/timing12", O_WRONLY);

  TIMER_CTRL = open("/dev/timing11", O_WRONLY);
  TIMER_1 = open("/dev/timing9", O_WRONLY);

  if ( DO_CSR < 1 || DO_FIFO < 1 ) {
    printf("Couldn't open do_csr\n");
    exit(3);
  }

  timer = 0x00;

  LSB_ONLY_8254(timer);
  MODE_2_8254(timer);
  BINARY_8254(timer);

  COUNTER_1_8254(timer);

  write(TIMER_CTRL, &timer, sizeof(char));
  write(TIMER_1, &clock, sizeof(char));

  RESET_OCSR(cmd);
  WIDTH_32_OCSR(cmd);
  CLOCK_10MHZ_OCSR(cmd);
  PAT_GEN_OCSR(cmd);
  DISABLE_OCSR(cmd);
  TERM_OFF_OCSR(cmd);
  CLEAR_FIFO_OCSR(cmd);
  NO_WAIT_NAE_OCSR(cmd);
  NO_TRIG_OCSR(cmd);
  NO_TRIG_END_OCSR(cmd);
  CLEAR_UNDER_OCSR(cmd);
  NO_SHAKE_OCSR(cmd);

  write(DO_CSR, &cmd, sizeof(__u32));

  write(DO_FIFO, fifo, SIXTEEN_K);
  
  SAVE_FIFO_OCSR(cmd);
  ENABLE_OCSR(cmd);

  write(DO_CSR, &cmd, sizeof(__u32));
  
  close(DO_CSR);
  close(DO_FIFO);
  close(TIMER_CTRL);
  close(TIMER_1);
  close(PLX_9080);

  free(fifo);

  return 0;
}
