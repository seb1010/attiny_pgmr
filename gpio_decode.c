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
