#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/types.h>
#include "../include/do_csr.h"

#define SIXTEEN_K 16*1024

int main(void) {

  int DO_CSR;
  __u32 cmd;

  DO_CSR  = open("/dev/timing1", O_RDWR);
  if ( DO_CSR < 1 ) {
    printf("Couldn't open do_csr\n");
    exit(3);
  }

  RESET_OCSR(cmd);
  WIDTH_32_OCSR(cmd);
  CLOCK_10MHZ_OCSR(cmd);
  DISABLE_OCSR(cmd);
  CLEAR_FIFO_OCSR(cmd);

  write(DO_CSR, &cmd, sizeof(__u32));
  
  close(DO_CSR);

  return 0;
}
