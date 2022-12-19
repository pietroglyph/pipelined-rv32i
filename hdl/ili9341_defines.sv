// Based on defines from: https://github.com/adafruit/Adafruit_ILI9341
`ifndef ILI9341_DEFINES_H
`define ILI9341_DEFINES_H

// From Section 8.1 of the Datasheet

typedef enum logic [7:0] {
  NOP = 8'h00,       // no-op register (do nothing)
  SWRESET = 8'h01,   // software reset
  RDDID = 8'h04,     // read display id info
  RDDST = 8'h09,     // read display status
  
  SLPIN = 8'h10,     // enter sleep mode
  SLPOUT = 8'h11,    // exist sleep mode
  PLTON = 8'h12,     // partial mode on
  NORON = 8'h13,     // normal mode on
  
  RDMODE = 8'h0A,    // read display power mode
  RDMADCTL = 8'h0B,  // read display MADCTL
  RDPIXFMT = 8'h0C,  // Read Display Pixel Format
  RDIMGFMT = 8'h0D,   // Read Display Image Format
  
  RDSELFDIAG = 8'h0F, // Read Display Self-Diagnostic Result
  INVOFF = 8'h20,   ///< Display Inversion OFF
  INVON = 8'h21,   ///< Display Inversion ON
  GAMMASET = 8'h26, ///< Gamma Set
  DISPOFF = 8'h28,  ///< Display OFF
  DISPON = 8'h29,   ///< Display ON

  CASET = 8'h2A, ///< Column Address Set
  PASET = 8'h2B, ///< Page Address Set
  RAMWR = 8'h2C, ///< Memory Write
  RAMRD = 8'h2E, ///< Memory Read

  PTLAR = 8'h30,    ///< Partial Area
  VSCRDEF = 8'h33,  ///< Vertical Scrolling Definition
  MADCTL = 8'h36,   ///< Memory Access Control
  VSCRSADD = 8'h37, ///< Vertical Scrolling Start Address
  PIXFMT = 8'h3A,   ///< COLMOD: Pixel Format Set

  FRMCTR1 = 8'hB1, ///< Frame Rate Control (In Normal Mode/Full Colors)
  FRMCTR2 = 8'hB2, ///< Frame Rate Control (In Idle Mode/8 colors)
  FRMCTR3 = 8'hB3, ///< Frame Rate control (In Partial Mode/Full Colors)
  INVCTR = 8'hB4,  ///< Display Inversion Control
  DFUNCTR = 8'hB6, ///< Display Function Control

  PWCTR1 = 8'hC0, ///< Power Control 1
  PWCTR2 = 8'hC1, ///< Power Control 2
  PWCTR3 = 8'hC2, ///< Power Control 3
  PWCTR4 = 8'hC3, ///< Power Control 4
  PWCTR5 = 8'hC4, ///< Power Control 5
  VMCTR1 = 8'hC5, ///< VCOM Control 1
  VMCTR2 = 8'hC7, ///< VCOM Control 2

  RDID1 = 8'hDA, ///< Read ID 1
  RDID2 = 8'hDB, ///< Read ID 2
  RDID3 = 8'hDC, ///< Read ID 3
  RDID4 = 8'hDD, ///< Read ID 4

  GMCTRP1 = 8'hE0, ///< Positive Gamma Correction
  GMCTRN1 = 8'hE1, ///< Negative Gamma Correction
  PWCTR6  = 8'hFC
} ILI9341_register_t;

typedef enum logic [15:0] {
  BLACK = 16'h0000,       ///<   0,   0,   0
  NAVY = 16'h000F,        ///<   0,   0, 123
  DARKGREEN = 16'h03E0,   ///<   0, 125,   0
  DARKCYAN = 16'h03EF,    ///<   0, 125, 123
  MAROON = 16'h7800,      ///< 123,   0,   0
  PURPLE = 16'h780F,      ///< 123,   0, 123
  OLIVE = 16'h7BE0,       ///< 123, 125,   0
  LIGHTGREY = 16'hC618,   ///< 198, 195, 198
  DARKGREY = 16'h7BEF,    ///< 123, 125, 123
  BLUE = 16'h001F,        ///<   0,   0, 255
  GREEN = 16'h07E0,       ///<   0, 255,   0
  CYAN = 16'h07FF,        ///<   0, 255, 255
  RED = 16'hF800,         ///< 255,   0,   0
  MAGENTA = 16'hF81F,     ///< 255,   0, 255
  YELLOW = 16'hFFE0,      ///< 255, 255,   0
  WHITE = 16'hFFFF,      ///< 255, 255, 255
  ORANGE = 16'hFD20,      ///< 255, 165,   0
  GREENYELLOW = 16'hAFE5, ///< 173, 255,  41
  PINK = 16'hFC18        ///< 255, 130, 198
} ILI9341_color_t;

`endif // ILI9341_DEFINES_H
