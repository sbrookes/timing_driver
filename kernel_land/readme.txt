
Design of the DMA routines.

Problem: Adlink PCIe-7300A does not interrupt the CPU when the FIFO buffer
	 is almost empty. This generates issues when the length of the desired
	 data output is greater than that of the FIFO itself.

Analysis: We will use the ability of the PLX 9080 PCI interface chip to 
	  interrupt the CPU when the DMA transfer to the FIFO has terminated. 
	  The output rate is configurable on the PCIe-7300A Digital IO card 
	  and so it is known. Knowing this rate will allow us to estimate 
	  how long after a given transfer completes the fifo will be 
	  "almost empty". "Almost empty" is programmable on the PCIe-7300A,
	  however this driver will ignore the officially registered "almost 
	  empty" value.  


     Almost Full ------+           Almost Empty -----+
      programmable     |            programmable     |
                       V                             V
             +---...---+--------------+--------------+---...---+
 +-------+   |         :              :              :         | +--------+
 | DMA   |   :         |    variable  :              |         : | output |
-+-------+-->|         :       |----->+              :         |-+--------+-->
 | speed |   :         |    width (w) :              |         : | speed  |
 +-------+   |         :              :              :         | +--------+
       	     +---...---+--------------+--------------+---...---+
	     |<------.....------  16k width  ------.....------>|



    NOTE: It is trivial to measure the time passage between the initiation
    	  of the DMA transfer and the "dma done" interrupt. A full
	  implementation can use this metric as well as the output speed
	  to change the parameters f, e, and adjust the estimated time for 
	  a full buffer to reach e. Careful monitoring could also predict
	  whether a slow DMA transfer and a fast output rate might necessitate
	  a DMA transfer > 16k to avoid interrupting the transfer as data
	  is continuously output.

    This implementation uses a somewhat simplified set of rules. It was made
    with reference to the following timeline. Note that the timeline is NOT
    drawn with any relevant sense of scale.

          
                 | -> time = 0         
        T |      |
        I |      | ---> first DMA transfer starts
        M |      |  :
        E |      |  : Tt1 (time to complete 1st transfer)
          V      |  :
                 | ---> first DMA transfer ends
                 |  :
                 |  :
                 |  : Od (output delay time)
                 |  :
                 |  :
                 | ---> output enabled
                 |  : 
                 |  :
                 |  : Wt (wait time, full -> almost empty)
                 |  :
                 |  :
            ---  | ---> almost empty, start 2nd DMA transfer
             :   |  : Tt2 (time to complete 2nd transfer)
             :   | ---> second DMA transfer ends.
          Wt :   |  :
             :   |  : Wt' = Wt - Tt2;
             :   |  :
            ---  | ---> almost empty, start 3rd dma transfer
             :   |  : Tt3 (time to complete 3nd transfer)
             :   | ---> second DMA transfer ends.
          Wt :   |  :
             :   |  : Wt'' = Wt - Tt3;
             :   |  :
            ---  | ---> almost empty, start 4th dma transfer
                 |
		 :
		 :
		 :
                 |
                 |
		 V


Results: Examining the timeline presented above, we can begin to generate
	 a diagnostic to dynamically determine Wt'(''...). Wt itself is 
	 a constant calculated with the "almost empty" definiton and the 
	 output rate (the frequency of the output clock). If Ttn is smaller
	 than Wt (as drawn) then {Wt'('*n) = Wt - Ttn} as shown. If, however,
	 Ttn is greater than or equal to Wt, then Wt'(''...) will always be 0.
	 In this case, there is an upper bound on the duration of output. If
	 Wt < Tt, the output FIFO will eventually be underrun. upper bound
	 should be great as the FIFO has sufficient depth.

	 Although the driver cannot decrease Ttn for a given transfer size, it 
	 can try to reduce the amount of time wasted between "almost empty" and
	 "nth dma transfer starts". The implementation specifically tries to 
	 keep this time small, and the time is not shown on the timeline above.
	 However, it is not 0. If Wt < Tt, a possible solution to avoid or
	 extend the upper bound problem is to initiate a DMA transfer with a 
	 size greater than the depth of the FIFO. This will reduce the latency
	 time built into Ttn on the timeline above, and will have the effect
	 of decreasing Tt. This is risky, however. One can imagine a case 
	 where an undue and temporary load on the PCI bus increases Ttn 
	 dramatically. The driver might initiate a very large DMA transfer to
	 try to compensate, but if the PCI bus becomes free and Ttn decreases, 
	 the driver now faces the potential for an overrun situation which is
	 just as bad as an underrun.


Implementation: The desired behavior was implemented with an interrupt handler
	 and a kernel thread. In a previous implementation, a kernel thread
	 was used with the following (undesireable) algorithm:

	          XXXXX OLD "polling" XXXXXXX
                  XXXXXXXX method XXXXXXXXXXX
                  X                         X
                  X  while(1){              X
                  X    if(dma_done)         X
                  X      start_new_xfer();  X
                  X    else                 X
                  X      pause              X
                  X  }                      X
                  X                         X
		  XXXXXXXXXXXXXXXXXXXXXXXXXXX

	Careful balance of the dma size can garuntee that there is no 
	over or underflow. This is undesireable because the benefit of DMA
	is that the system does work without passing burden onto the processor.
	With this polling thread, the processor is being taxed 100% of the time
	that DMA is being used anyway, largely defeating the purpose.

	In the current implentation, the kernel thread follows the following
	algorithm:

                  +-------- kthread --------+
		  |                         |
		  | while(1){               |
		  |                         |
		  |   wait(Wt');            |
		  |                         |
		  |   start_new_xfer();     |
		  |                         |
		  |   sleep_until_awoken(); |
		  |                         |
                  | }                       |
		  |                         |
                  +-------------------------+

        while the interrupt handler follows the following algorithm.

                  +--- interrupt handler ---+
		  |                         |
		  | calculate Wt'           |
		  |                         |
		  | wake_up_kthread();      |
		  |                         |
                  +-------------------------+
	      
