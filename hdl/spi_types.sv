`ifndef SPI_TYPES_H
`define SPI_TYPES_H

typedef enum logic [2:0] {
  WRITE_8, WRITE_16, WRITE_8_READ_8, WRITE_8_READ_16, WRITE_8_READ_24
} spi_transaction_t;

`endif // SPI_TYPES_H