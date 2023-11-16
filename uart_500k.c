/*
Needs following macro definitions
  UART_TX
  UART_RX
  MAX_PACKET_LEN
  
need the variable definitons for
  input_buffer
  output_buffer 
  
  only works for porta uarts
  requires 8MHz clock speed

  but hey at least there are no dependencies
*/


static uint8_t interrupt_happened = 0;

// returns the two-compliment of the checksum
// for rx this will be 0 on success
// for tx this will be the value to append
int8_t check_sum(uint8_t* buffer, uint8_t num_bytes){
  uint8_t i, sum=0;

  for(i=0; i<num_bytes; i++){
    sum += *(buffer + i);
  }
  sum = ~(sum) + 1;

  return sum;
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


void send_one_uart_byte(uint8_t data){
  *(output_buffer) = 3; 
  *(output_buffer + 1) = data;
  *(output_buffer + 2) = check_sum(output_buffer, 2);
  send_uart_packet();
}

ISR(TIM0_COMPA_vect){
 interrupt_happened = 1;
}

int8_t wait_and_read_packet(uint8_t timeout){
  // waits for a packet (will wait forever)
  // reads the packet into memory
  // returns 0 on success
  uint8_t packet_len;
  uint8_t bytes_read;
  int8_t packet_read_status;
  uint8_t interrupt_enabled = SREG & (1 << 7);


  if(timeout){
    TCNT0 = 0; // clear counter
    TCCR0A = 0; // normal mode
    TCCR0B = (1 << CS02) | (1 << CS00); //clk / 1024
    OCR0A = timeout; //compare A to timeout
    TIMSK0 |= (1 << OCIE0A);
    TIFR0 |= (1 << OCF0A); // ensuring flag is clear
    sei();
    interrupt_happened = 0;
  }
  asm volatile(
  // setting up pointer register
    // automatically done
  // main loop
  "clr %[bytes_read] \n\t"
  "clr %[packet_len] \n\t"
  "clr %[err] \n\t"
  "uart_byte_loop: \n\t"

    //wait for next byte
    "wait_for_byte: \n\t"
      "in r17, %[port] \n\t"
      "ld r22, %a[int_ptr] \n\t"
      "cbr r17, 0xFF & ~(1 << %[RX_PIN]) \n\t"
      "sbrs r22, 0 \n\t"
      "brne wait_for_byte \n\t"

    // lets note if we timed out   
    "cpi r22, 0x01 \n\t"
    "brne no_timeout \n\t"
      "ldi %[err], (%[TIMEOUT_ERROR]) \n\t"
      "rjmp end_read_loop \n\t"
    "no_timeout: \n\t"

    //making sure there is memory left
    "cpi %[bytes_read], %[MAX_LEN] \n\t"
    "brcs no_overflow \n\t"
      "ldi %[err], %[OVERFLOW_ERROR] \n\t"
      "clr %[bytes_read] \n\t"
      "rjmp end_byte_loop \n\t"
    "no_overflow: \n\t"

    // delay before first bit
/*    "nop \n\t"
    "nop \n\t"
    "nop \n\t"
*/
    //start read loop
    "clr r18 \n\t"
    "read_loop: \n\t"
    "cpi r18, 0x08 \n\t"         // 1 cycle
    "inc r18 \n\t"               // 1 cycle
    "brcc end_read_loop \n\t"    // 2 cycles
//"sbi 0x19, 7 \n\t"
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
    "st %a[ptr]+, r19 \n\t"
    "inc %[bytes_read] \n\t" // inc the byte counter

    // if this is the first byte set r21 to packet len
    "cpi %[bytes_read], 0x01 \n\t"
    "brne not_first_byte \n\t"
      "mov %[packet_len], r19 \n\t"
    "not_first_byte: \n\t"

    // if we have read all the bytes let's exit
    "cp %[bytes_read], %[packet_len] \n\t"
      "brcc end_byte_loop \n\t"

    // reseting the timeout with each byte
    "clr r17 \n\t"
    "out %[TCNT0_S], r17 \n\t"

    "rjmp uart_byte_loop \n\t"
    "end_byte_loop: \n\t"

    : [err]"=&d" (packet_read_status),   // output
      [bytes_read]"=&d" (bytes_read),
      [packet_len]"=&d" (packet_len)
    : [ptr]"e" (input_buffer),        //input
      [port]"I" (_SFR_IO_ADDR(PINA)),
      [RX_PIN]"M" (UART_RX),
      [MAX_LEN]"M" (MAX_PACKET_LEN),
      [OVERFLOW_ERROR]"M" (OVERFLOW_ERROR_SEB),
      [TIMEOUT_ERROR]"M" (TIMEOUT_ERROR_SEB),
      [TCNT0_S]"M" (_SFR_IO_ADDR(TCNT0)),
      [int_ptr]"e" (&interrupt_happened)
    : "r17", "r18", "r19", "r20", "r21", "r22"     // cobblers
  );

  // just reseting things if timeout was enabled
  if(timeout){
    TCCR0B = 0; // stops timer clock
    TIMSK0 &= ~(1 << OCIE0A); // mask compare
    if(!interrupt_enabled)
      cli();
  }

  // if we we overflow lets pretend we didn't read anything
  if(packet_read_status){
    //*input_buffer = 0;
  }else if (check_sum(input_buffer, *input_buffer)){
    packet_read_status = CHECKSUM_ERROR;
  }

  return packet_read_status;
}

// waits for ack
uint8_t send_assured_packet(uint8_t num_tries, uint8_t timeout){
  uint8_t i = 0;
  int8_t ack_missing = 1;

  while(i<num_tries && ack_missing){
    i++;
    send_uart_packet();
    ack_missing = wait_and_read_packet(timeout);
    if(!ack_missing && (*(input_buffer + 1) == ACK_PID)){
        ack_missing = 0;
    }else{
      ack_missing = UNEXPECTED_PACKET_ERROR;
    }
  }

  return ack_missing;
}
