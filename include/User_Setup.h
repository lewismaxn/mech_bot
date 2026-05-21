// ===================================================
// TFT_eSPI — User_Setup.h for LilyGO T-Display S3
//
// ST7789 driver, 8-bit parallel interface, 170×320 px
// Shared by both master and slave T-Display S3 boards.
//
// Pin source: Context/wiring_and_components.md
// ===================================================

// Suppress the library's default setup
#define USER_SETUP_LOADED

// ─── Driver ────────────────────────────────────────
#define ST7789_DRIVER

// ─── Display dimensions (portrait) ─────────────────
#define TFT_WIDTH  170
#define TFT_HEIGHT 320

// ─── 8-bit parallel interface ───────────────────────
#define TFT_PARALLEL_8_BIT

// Control pins
#define TFT_CS    6
#define TFT_DC    7
#define TFT_RST   5
#define TFT_WR    8
#define TFT_RD    9

// Data bus D0–D7
#define TFT_D0   39
#define TFT_D1   40
#define TFT_D2   41
#define TFT_D3   42
#define TFT_D4   45
#define TFT_D5   46
#define TFT_D6   47
#define TFT_D7   48

// Backlight is managed manually in code: pinMode(15, OUTPUT); digitalWrite(15, HIGH);

// ─── Fonts ─────────────────────────────────────────
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

// ─── SPI speed (not used for parallel, required by lib) ─
#define SPI_FREQUENCY  40000000
