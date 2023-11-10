uint8_t check_sum(uint8_t* buffer, uint8_t num_bytes){
  uint8_t i, sum=0;

  for(i=0; i<num_bytes; i++){
    sum += *(buffer + i);
  }

  sum = ~(sum) + 1;

  return sum;
}

void send_one_uart_byte(uint8_t data){
  *(output_buffer) = 3; 
  *(output_buffer + 1) = data;
  *(output_buffer + 2) = check_sum(output_buffer, 2);
  send_uart_packet();
}

#define NUM_UART_BITS 10 // 8 + start + stop
void send_uart_packet(){
  uint8_t i, bit_num;
  uint8_t* current_loc;
  uint16_t data_byte; // extra bits for start and stop

  uint8_t porta_val = PORTA;

  for(i=0; i < *output_buffer; i++){
    current_loc = output_buffer + i;
    bit_num = 0;
    data_byte = (((*current_loc) | 0xFF00) << 1);
 //   seb_print_v2("byte_out %x", (uint32_t)data_byte);
    asm volatile(
      // loop stuff
      "start_bit_loop: \n\t"
        "cpi %[bit_num], %[NUM_BITS] \n\t"
        "brcc end_uart_bit_loop \n\t"
        "inc %[bit_num] \n\t"

        // setting bits
        "cbr %[port_val], (1 << %[TX_PIN]) \n\t"
        "sbrc %A[data], 0 \n\t"
          "sbr %[port_val], (1 << %[TX_PIN]) \n\t"
        "out %[port_addr], %[port_val] \n\t"

        // preping next bit
        "lsr %B[data] \n\t"
        "ror %A[data] \n\t"

        // wait
        "nop \n\t"
        "nop \n\t"
        "nop \n\t"
        "nop \n\t"
        "nop \n\t"

      // looping things
      "rjmp start_bit_loop \n\t"
      "end_uart_bit_loop: \n\t"

    :
    :[port_addr]"M" (_SFR_IO_ADDR(PORTA)),
     [port_val]"r" (porta_val),
     [pointer]"e" (current_loc),
     [TX_PIN]"M" (UART_TX),
     [NUM_BITS]"M" (NUM_UART_BITS),
     [bit_num]"r" (bit_num),
     [data]"r" (data_byte)
    :
    );
  }

}


uint8_t wait_and_read_packet(){
  // waits for a packet (will wait forever)
  // reads the packet into memory
  // returns 0 on success
  uint8_t packet_len = 0;
  uint8_t bytes_read = 0;
  uint8_t packet_read_status = 0;

  asm volatile(

  // setting up pointer register
    // automatically done

  // main loop
  "uart_byte_loop: \n\t"

    //wait for next byte
    "wait_for_byte: \n\t"
      "in r17, %[port] \n\t"
      "cbr r17, 0xFF & ~(1 << %[RX_PIN]) \n\t"
      "brne wait_for_byte \n\t"

    //making sure there is memory left
    "cpi %[bytes_read], %[MAX_LEN] \n\t"
    "brcs no_overflow \n\t"
      "mov %[err], %[bytes_read] \n\t" // set error
      "clr %[bytes_read] \n\t"
      "rjmp end_byte_loop \n\t"
    "no_overflow: \n\t"

    // delay before first bit
    "ldi r20, 0x03 \n\t"
    "delay_loop_0: \n\t"
      "dec r20 \n\t"
      "brne delay_loop_0 \n\t"

    //start read loop
    "clr r18 \n\t"
    "read_loop: \n\t"
    "cpi r18, 0x08 \n\t"         // 1 cycle
    "inc r18 \n\t"               // 1 cycle
    "brcc end_read_loop \n\t"    // 2 cycles
      "in r17, %[port] \n\t"     // 2 cycles
      "lsr r19 \n\t"             // 1 cycle
      "sbrc r17, %[RX_PIN] \n\t" // 1 cycle
        "sbr r19, (1 << 7) \n\t" // 1 cycle
       "nop \n\t"
       "ldi r20, 0x02 \n\t"          // 1 cycle
       "delay_loop: \n\t"         // 3n cycles
       "dec r20 \n\t"
       "brne delay_loop \n\t"
      "rjmp read_loop \n\t"    // 2 cycles
    
    "end_read_loop: \n\t"

    // store the byte we just captured
    "st %a[ptr], r19 \n\t"
    "inc %[bytes_read] \n\t" // inc the byte counter

    // if this is the first byte set r21 to packet len
    "cpi %[bytes_read], 0x01 \n\t"
    "brne not_first_byte \n\t"
      "mov %[packet_len], r19 \n\t"
    "not_first_byte: \n\t"

    // if we have read all the bytes let's exit
    "cp %[bytes_read], %[packet_len] \n\t"
      "brcc end_byte_loop \n\t"

    "rjmp uart_byte_loop \n\t"
    "end_byte_loop: \n\t"

    : [err]"=&r" (packet_read_status),   // output
      [bytes_read]"+d" (bytes_read),
      [packet_len]"+d" (packet_len)
    : [ptr]"e" (input_buffer),        //input
      [port]"I" (_SFR_IO_ADDR(PINA)),
      [RX_PIN]"M" (UART_RX),
      [MAX_LEN]"M" (MAX_PACKET_LEN)
    : "r17", "r18", "r19", "r20"     // cobblers
  );

  // if we we overflow lets pretend we didn't read anything
  if(packet_read_status)
    *input_buffer = 0;

  return packet_read_status;
}
