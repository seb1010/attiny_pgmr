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
#define CLK_FREQ 8000000UL


uint8_t input_buffer[MAX_PACKET_LEN];
uint8_t output_buffer[MAX_PACKET_LEN];
uint8_t input_offset = 0;

#include "attiny_programmer_mk2/uart_500k.c"
#include "utils/seb_print_v3.c"

struct pgmr_config{
  uint8_t spi_per;
  uint8_t and_read_spi;
  uint8_t prog_enabled;
};

uint8_t read_write_spi_byte(uint8_t, uint8_t);
void pass_input_to_spi(uint8_t spi_per);
int8_t config_pgmr(struct pgmr_config *pgc);
int8_t config_dut(struct pgmr_config *dut_phy);
int8_t get_pgmr_status(struct pgmr_config *pcg);
int8_t get_gpio_info(uint8_t, volatile uint8_t**,
                     volatile uint8_t**, volatile uint8_t*);

int main (){
  // configure io
  // starts with programming disabled
  PORTA = (1 << UART_TX) | (1 << RESET_PIN);
  DDRA = (1 << UART_TX) | (1 << RESET_PIN);


  int8_t packet_bad;
  int8_t instr_fail;
  uint8_t send_ack;
  uint8_t bad_checksums = 0;

  struct pgmr_config pgmr_phy;
  pgmr_phy.spi_per = 10;
  pgmr_phy.and_read_spi = 1;
  pgmr_phy.prog_enabled = 0;

  while(1){
      // reading with no timeout
      packet_bad = wait_and_read_packet(0);
     
      // if checksum looks good lets decode
      send_ack = 1;
      if(!packet_bad){
        bad_checksums = 0;
        switch( *(input_buffer + 1)){
          case DATA_PID: // data packet to passthrough
            if(pgmr_phy.and_read_spi){
              send_one_uart_byte(ACK_PID);
              pass_input_to_spi(pgmr_phy.spi_per);
              *(output_buffer + (*output_buffer) - 1) =
                  check_sum(output_buffer, (*output_buffer) - 1);
              send_uart_packet();
              send_ack = 0;
            }else{
              pass_input_to_spi(pgmr_phy.spi_per);
            }
            break;
          case CONFIG_PGMR_PID: // configure pgmr
            instr_fail = config_pgmr(&pgmr_phy);
            break;
          case CONFIG_DUT_PID: // configure dut
            instr_fail = config_dut(&pgmr_phy);
            break;
          case GET_PGMR_STATUS_PID: // configure dut
            send_one_uart_byte(ACK_PID);
            send_ack = 0;
            instr_fail = get_pgmr_status(&pgmr_phy);
            break;
          case ACK_PID: // debugging only?
              send_ack = 0;
              instr_fail = 0;
              break;
          default:
            seb_print_v3("err %d", (uint32_t)(*(input_buffer + 1)));
        }
        if(instr_fail)
            send_one_uart_byte(NAK_PID);
        else if (send_ack)
            send_one_uart_byte(ACK_PID);

      }else{
        // if here bad checksum
        bad_checksums++;
        if (bad_checksums < 2){
          send_one_uart_byte(NAK_PID);
        }else{
          // if multiple checksum errors we likely have a sync issue
          // in order to resync we run read_packet with a timeout
          // and we just keep running that until we get a good packet
          while (packet_bad){
            packet_bad = wait_and_read_packet(200);
          }
        }
      }

    }

  return 0;
}

int8_t config_pgmr(struct pgmr_config *pgc){
  int8_t decode_fail = 0;
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
  return decode_fail;
}

int8_t get_pgmr_status(struct pgmr_config *pgc){
  *output_buffer = 0x06;
  *(output_buffer + 1) = DATA_PID;
  *(output_buffer + 2) = (*pgc).spi_per;
  *(output_buffer + 3) = (*pgc).and_read_spi;
  *(output_buffer + 4) = (*pgc).prog_enabled;
  *(output_buffer + 5) = check_sum(output_buffer, (*output_buffer) - 1);

  send_uart_packet();
  return 0; // eventually send assured packet
}


int8_t enable_programming(struct pgmr_config *pgc){
  int8_t ret;
  uint8_t spi_per = (*pgc).spi_per;

  // len pid_config_pgmr enable_proig, true/false
  if(*(input_buffer + 3)){
    // pins to dut to output and correct state
    DDRA |= (1 << SPI_MOSI_PIN) | (1 << SPI_SCLK_PIN);
    PORTA &= ~((1 << RESET_PIN) | (1 << SPI_MOSI_PIN) | (1 << SPI_SCLK_PIN));

    // send reset pulse
    _delay_loop_2(0xFFFF);
    PORTA |= (1 << RESET_PIN);
    _delay_loop_2(0xFFFF);
    PORTA &= ~(1 << RESET_PIN);
    _delay_loop_2(0xFFFF);

    // send programming enable command
    *input_buffer = 7;
    *(input_buffer + 2) = 0xac;
    *(input_buffer + 3) = 0x53;
    *(input_buffer + 4) = 0x00;
    *(input_buffer + 5) = 0x00;
    pass_input_to_spi(spi_per);
  
    // check if programming was enabled successfully
    if(*(output_buffer + 4) == 0x53){
      ret = 0;

      //update programer status
      (*pgc).prog_enabled = 0x01;
    }else{
      ret = OPERATION_FAILED_PID;

      // make sure sure everyone knows prog is not enabled
      (*pgc).prog_enabled = 0x00;

      // set the pgmr config to match not enabled
      DDRA &= ~((1 << SPI_MOSI_PIN) | (1 << SPI_SCLK_PIN));
      PORTA |= (1 << RESET_PIN);
    }
  }else{
    DDRA &= ~((1 << SPI_MOSI_PIN) | (1 << SPI_SCLK_PIN));
    PORTA |= (1 << RESET_PIN);

    // update the pgmr status
    (*pgc).prog_enabled = 0x00;
    ret = 0;
  }
  return ret;
}

//len, conf_dut, specific conf, gpio num, gpio val
int8_t config_dut(struct pgmr_config *pgmr_phy){
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
      decode_fail = enable_programming(pgmr_phy);
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


      // set clock low set MOSI
      "cbr %[port_val], (1 << %[SCLK]) | (1 << %[MOSI]) \n\t"
      "sbrc %[data], 7 \n\t"
        "sbr %[port_val], (1 << %[MOSI]) \n\t"
      "out %[port_addr], %[port_val] \n\t"
      "lsl %[data] \n\t"

      // for (i=0; i<8; i++)
      "cpi %[bit_num], %[num_bits] \n\t"
      "inc %[bit_num] \n\t"
      "brcc end_spi_bit_loop \n\t"

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


// returns -1 on fail
int8_t get_gpio_info(uint8_t gpio_num,
                     volatile uint8_t** port_num,
                     volatile uint8_t** gpio_ddr,
                     volatile uint8_t* pin_num){
  int8_t ret = 0;
  switch (gpio_num){
    case 0:
      *pin_num = 6;
      *port_num = &PORTA;
      *gpio_ddr = &DDRA;
      break;
    case 1:
      *pin_num = 7;
      *port_num = &PORTA;
      *gpio_ddr = &DDRA;
      break;
    case 2:
      *pin_num = 0;
      *port_num = &PORTB;
      *gpio_ddr = &DDRB;
      break;
    case 3:
      *pin_num = 1;
      *port_num = &PORTB;
      *gpio_ddr = &DDRB;
      break;
    case 4:
      *pin_num = 2;
      *port_num = &PORTB;
      *gpio_ddr = &DDRB;
      break;
    case 5:           // reset
      *pin_num = 5;
      *port_num = &PORTA;
      *gpio_ddr = &DDRA;
      break;
    default:
      ret = -1;
  }
  return ret;
}
