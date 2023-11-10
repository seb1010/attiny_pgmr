#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay_basic.h>
#include "attiny_programmer_mk2/defs.h"


#define GPIO_1 7
#define GPIO_2 0
#define GPIO_3 1
#define GPIO_4 2
#define UART_RX 1
#define UART_TX 0
#define SPI_MISO_PIN 2
#define SPI_MOSI_PIN 3
#define SPI_SCLK_PIN 4
#define RESET_PIN 5

#define DEVICE_ID 0x00
#define DEVICE_TYPE 0x00
#define BAUD_RATE 9600
#define CLK_FREQ 8000000UL
#define MAX_PACKET_LEN 66 // 64 + pid + checksum


//#include "utils/packet_util.c"
//#include "utils/seb_print_v2.c"

uint8_t input_buffer[MAX_PACKET_LEN];
uint8_t output_buffer[MAX_PACKET_LEN];
uint8_t input_offset = 0;

#include "utils/uart_500k.c"
#include "utils/seb_print_v3.c"
#include "attiny_programmer_mk2/gpio_decode.c"

//void send_one_uart_byte(uint8_t);
//uint8_t wait_and_read_packet();
//uint8_t check_sum(uint8_t*, uint8_t);
//void send_uart_packet();

struct pgmr_config{
  uint8_t spi_per;
  uint8_t and_read_spi;
};

uint8_t read_write_spi_byte(uint8_t, uint8_t);
void pass_input_to_spi(uint8_t spi_per);
uint8_t config_pgmr(struct pgmr_config *pgc);
uint8_t config_dut(struct pgmr_config *dut_phy);

int main (){
  // configure io
  DDRA = (1 << UART_TX)      |
         (1 << SPI_MOSI_PIN) |
         (1 << RESET_PIN)    |
         (1 << SPI_SCLK_PIN);  // bus contention
  PORTA |= (1 << UART_TX);
  DDRB |= (1 << GPIO_2);

//  configure_uart();
 
  uint32_t i;
  uint8_t ret;
  uint8_t packet_bad;
  uint8_t instr_fail;

  struct pgmr_config pgmr_phy;
  pgmr_phy.spi_per = 10;
  pgmr_phy.and_read_spi = 1;

  while(1){
      packet_bad = wait_and_read_packet(0);
     
      //seb_print_v3("byte0: %x", (uint32_t)(*input_buffer));

     //seb_print_v3("readin ", 0); 
     //for(i=0; i<MAX_PACKET_LEN; i++)
     //     *(output_buffer + i) = *(input_buffer + i);
     //send_uart_packet();

      // if checksum looks good lets decode
      if(!packet_bad){
        switch( *(input_buffer + 1)){
          case DATA_PID: // data packet to passthrough
            pass_input_to_spi(pgmr_phy.spi_per);
            if(pgmr_phy.and_read_spi);
              // send some packet
            send_one_uart_byte(ACK_PID);
            break;
          case CONFIG_PGMR_PID: // configure pgmr
            instr_fail = config_pgmr(&pgmr_phy);
            seb_print_v3("c pgmr det", 0);
            break;
          case CONFIG_DUT_PID: // configure dut
            instr_fail = config_dut(&pgmr_phy);
            break;
          default:
            seb_print_v3("err %d", (uint32_t)(*(input_buffer + 1)));
        }
        if(instr_fail)
            send_one_uart_byte(NAK_PID);
        else
            send_one_uart_byte(ACK_PID);

      }else{
        send_one_uart_byte(NAK_PID);
        seb_print_v3("bad pa co %x", (uint32_t)packet_bad);
      }

    }

  return 0;
}

uint8_t config_pgmr(struct pgmr_config *pgc){
  uint8_t decode_fail = 0;
  switch (*(input_buffer + 2)){
    case SET_SPI_RATE_PID: // set spi rate
      (*pgc).spi_per = *(input_buffer + 3);
      break;
    case READ_SPI_PID:
      (*pgc).and_read_spi = *(input_buffer + 3);
      break;
    default:
      decode_fail = 1;
  }
  seb_print_v3("spi_p: %d", (uint32_t)((*pgc).spi_per));
  return decode_fail;
}


int8_t enable_programming(uint8_t spi_per){
  int8_t ret;

  PORTA &= ~((1 << RESET_PIN) | (1 << SPI_MOSI_PIN) | (1 << SPI_SCLK_PIN));
  _delay_loop_2(0xFFFF);
  PORTA |= (1 << RESET_PIN);
  _delay_loop_2(0xFFFF);
  PORTA &= ~(1 << RESET_PIN);
  _delay_loop_2(0xFFFF);
  *input_buffer = 7;
  *(input_buffer + 2) = 0xac;
  *(input_buffer + 3) = 0x53;
  *(input_buffer + 4) = 0x00;
  *(input_buffer + 5) = 0x00;

  pass_input_to_spi(spi_per);

  if(*(output_buffer + 4) == 0x53)
    ret = 0;
  else
    ret = OPERATION_FAILED_PID;

  return ret;
}

//len, conf_dut, specific conf, gpio num, gpio val
uint8_t config_dut(struct pgmr_config *pgmr_phy){
  int8_t decode_fail = 0;
  uint8_t gpio_pin;
  volatile uint8_t* gpio_port;
  volatile uint8_t* gpio_ddr;
  volatile uint8_t* port;

  //gpio_pin = 7;
  switch(*(input_buffer + 2)){
    case SET_DD_PID: // set gpio ddr
      decode_fail = get_gpio_info(*(input_buffer + 3),
         &gpio_port, &port, &gpio_pin);
      break;
    case SET_VAL_PID: // set gpio val
      decode_fail = get_gpio_info(*(input_buffer + 3),
          &port, &gpio_ddr, &gpio_pin);
      break;
    case ENABLE_PROGRAMMING_PID:
      decode_fail = enable_programming((*pgmr_phy).spi_per);
      break;
    default:
      decode_fail = 1;
  }
  if((!decode_fail)) {
    if(*(input_buffer + 4)){
      *port |= (1 << gpio_pin);
    }else{
      *port &= ~(1 << gpio_pin);
    }
  }

  return decode_fail;
}

void pass_input_to_spi(uint8_t spi_per){
  uint8_t i;
  *output_buffer = *input_buffer;
  *(output_buffer + 1) = DATA_PID;
  for(i=2; i<(*input_buffer) - 1; i++){
    *(output_buffer + i) = read_write_spi_byte(*(input_buffer + i), spi_per);
  }
}

// writes a bit at the given speed
// returns the byte read during the write
uint8_t read_write_spi_byte(uint8_t data, uint8_t per_delay){
  uint8_t byte_read;
  uint8_t porta_val = PORTA;
  uint8_t pin_val;
  uint8_t delay_counter;
  uint8_t bit_num = 0;

  asm volatile(

    // for (i=0; i<8; i++)
    "spi_bit_loop: \n\t"
      "cpi %[bit_num], %[num_bits] \n\t"
      "inc %[bit_num] \n\t"
      "brcc end_spi_bit_loop \n\t"


      // set clock low set MOSI
      "cbr %[port_val], (1 << %[SCLK]) | (1 << %[MOSI]) \n\t"
      "sbrc %[data], 7 \n\t"
        "sbr %[port_val], (1 << %[MOSI]) \n\t"
      "out %[port_addr], %[port_val] \n\t"
      "lsl %[data] \n\t"

      // delay for a bit
      "mov %[i], %[delay] \n\t"
      "delay_loop_7: \n\t"
        "dec %[i] \n\t"
        "brne delay_loop_7 \n\t"

      // read MISO
      "in %[pin_val], %[pin_addr] \n\t"
      "lsl %[byte_read] \n\t"
      "sbrc %[pin_val], %[MISO] \n\t"
        "inc %[byte_read] \n\t" //just setting bit 0

      // sclk high
      "sbr %[port_val], (1 << %[SCLK]) \n\t"
      "out %[port_addr], %[port_val] \n\t"

      // delay for a bit
      "mov %[i], %[delay] \n\t"
      "delay_loop_8: \n\t"
        "dec %[i] \n\t"
        "brne delay_loop_8 \n\t"

    "rjmp spi_bit_loop \n\t" // end of for loop
    "end_spi_bit_loop: \n\t"
  
  : [byte_read]"=&d" (byte_read),
    [i]"=&r" (delay_counter),
    [pin_val]"=&r" (pin_val)
  : [delay]"r" (per_delay),
    [data]"r" (data),
    [port_val]"d" (porta_val),
    [port_addr]"M" (_SFR_IO_ADDR(PORTA)),
    [pin_addr]"M" (_SFR_IO_ADDR(PINA)),
    [MOSI]"M" (SPI_MOSI_PIN),
    [MISO]"M" (SPI_MISO_PIN),
    [SCLK]"M" (SPI_SCLK_PIN),
    [bit_num]"r" (bit_num),
    [num_bits]"M" (8)
  );

  return byte_read;
}
