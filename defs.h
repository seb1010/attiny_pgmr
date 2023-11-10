// pids
#define ACK_PID 0x2d
#define NAK_PID 0xa5
#define DATA_PID 0x3c

#define CONFIG_PGMR_PID 0x4d
  #define SET_SPI_RATE_PID 0x01
  #define READ_SPI_PID 0x02

#define CONFIG_DUT_PID 0x42
  #define SET_DD_PID 0x1a
  #define SET_VAL_PID 0x1b
  #define ENABLE_PROGRAMMING_PID 0x05

// errors
#define OPERATION_FAILED_PID (0x100-5)
#define UNEXPECTED_PACKET_ERROR (0x100 - 4)//comment
#define TIMEOUT_ERROR_SEB (0x100 - 3)
#define OVERFLOW_ERROR_SEB (0x100 - 2)
#define CHECKSUM_ERROR (0x100 - 1)

