///ttn
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { FILLMEIN };
  void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
  }

  // This should also be in little endian format, see above.
  static const u1_t PROGMEM DEVEUI[8] = { FILLMEIN };
  void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
  }

  // This key should be in big endian format (or, since it is not really a
  // number but a block of memory, endianness does not really apply). In
  // practice, a key taken from the TTN console can be copied as-is.
  static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };


int sleepTime = 240000;
