/************************************************************************
*   Vector Interrupts                                                   *
************************************************************************/
wire    [7:0]       intsLatch;
wire    [7:0]       encoderIn;


//assign  intsIn[0] = 1'b1;
assign  intsIn[1] = 1'b1;
assign  intsIn[2] = rtc_n_INT;      // real time clock interrupt from RTC
assign  intsIn[3] = in_n_intD;      // INT_D-
assign  intsIn[4] = in_n_intC;       // INT_C-
assign  intsIn[5] = in_n_intB;       // INT_B-
assign  intsIn[6] = in_n_intA;       // INT_A-
assign  intsIn[7] = 1'b1;

assign  encoderIn[0] = intsLatch[7];
assign  encoderIn[7:1] = intsLatch[6:0];

assign intsToCpu[2:0] = 3'b0;
assign intsToCpu[7:6] = 2'b0;


n_bitLatch      #(8)
      intptlatch(
     .load      (n_inta),       // high-to-low transition
     .clock     (n_inta),
//     .clr       (1'b0),
     .inData    (intsIn),
     .regOut    (intsLatch)
     );

priEncoder priorityEncoder(
//    .encIn      (1'b0),
    .aIn        (~encoderIn),
    .encOut     (intsToCpu[5:3]),
    .intsGs     (intsGs)
    );
