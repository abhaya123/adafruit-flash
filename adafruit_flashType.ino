/*
 *  Retrieve basic core info of Flash SPI
 *  Add a custom device w25x80
 *  library Adafruit_SPIFlash and SdFat - AdafruitFork
 *
 *  by Mischianti Renzo <https://mischianti.org>
 *
 *  https://mischianti.org/
 *
 *  SPIFlash connected via SPI standard check wiring on the article
 *
 */
 

#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>
#define BUFSIZE   4096

// 4 byte aligned buffer has best result with nRF QSPI
uint8_t bufwrite[BUFSIZE] __attribute__ ((aligned(4)));
uint8_t bufread[BUFSIZE] __attribute__ ((aligned(4)));

Adafruit_FlashTransport_SPI flashTransport(SS,SPI); // Set CS and SPI interface
Adafruit_SPIFlash flash(&flashTransport);
 
void setup()
{
  Serial.begin(115200);
  while ( !Serial ) delay(100);   // wait for native usb
  SPI.setFrequency(12000000);
  
  #define W25Q128BV                                                            \
  {                                                                            \
    .total_size = (16UL << 20), /* 1 MiB */                                     \
        .start_up_time_us = 5000, .manufacturer_id = 0xef,                     \
    .memory_type = 0x40, .capacity = 0x18, .max_clock_speed_mhz = 104,         \
    .quad_enable_bit_mask = 0x02, .has_sector_protection = false,              \
    .supports_fast_read = true, .supports_qspi = true,                         \
    .supports_qspi_writes = true, .write_status_register_split = false,        \
    .single_status_byte = false, .is_fram = false,                             \
  }
 
 
static const SPIFlash_Device_t possible_devices[] = {
        W25Q128BV,
};
 
  if (flash.begin(possible_devices)) {
      Serial.println(F("Device finded and supported!"));
  } else {
      Serial.println(F("Problem to discover and configure device, check wiring also!"));
  }
  // Set 4Mhz SPI speed
   //flashTransport.setClockSpeed(4000000, 4000000); // added to prevent speed problem
    flashTransport.setClockSpeed(24000000, 24000000);

     Serial.println();

    Serial.println("Adafruit Serial Flash get basic info: ");
    Serial.print("JEDEC ID (FFFFFF for unknown): "); Serial.println(flash.getJEDECID(), HEX);
    Serial.print("Flash size: "); Serial.println(flash.size());
 
//  Serial.println();Serial.println();
// 
//  uint8_t jedec_ids[4];
//  flashTransport.readCommand(SFLASH_CMD_READ_JEDEC_ID, jedec_ids, 4);
//
////  // For simplicity with commonly used device, we only check for continuation
////  // code at 2nd byte (e.g Fujitsu FRAM devices)
//  if (jedec_ids[1] == 0x7F) {
//    // Shift and skip continuation code in 2nd byte
//    jedec_ids[1] = jedec_ids[2];
//    jedec_ids[2] = jedec_ids[3];
//  }
// 
//  Serial.println("Retrieve JDEC_ID");
// 
//  Serial.print("Manufacturer ID: 0x");
//  Serial.println(jedec_ids[0], HEX);
// 
//  Serial.print("Memory Type: 0x");
//  Serial.println(jedec_ids[1], HEX);
// 
//  Serial.print("Capacity: 0x");
//  Serial.println(jedec_ids[2], HEX);
//  Serial.print("Capacity DEC: ");
//  Serial.println(jedec_ids[2], DEC);

  Serial.flush();
 
  write_and_compare(0xAA);
  write_and_compare(0x55);
 
  Serial.println("Speed test is completed.");
  Serial.flush();
}

  void print_speed(const char* text, uint32_t count, uint32_t ms)
{
  Serial.print(text);
  Serial.print(count);
  Serial.print(" bytes in ");
  Serial.print(ms / 1000.0F, 2);
  Serial.println(" seconds.");
 
  Serial.print("Speed : ");
  Serial.print( (count / 1000.0F) / (ms / 1000.0F), 2);
  Serial.println(" KB/s.\r\n");
}
 
bool write_and_compare(uint8_t pattern)
{
  uint32_t ms;
 
  Serial.println("Erase chip");
  Serial.flush();
 
#define TEST_WHOLE_CHIP
 
#ifdef TEST_WHOLE_CHIP
  uint32_t const flash_sz = flash.size();
  flash.eraseChip();
#else
  uint32_t const flash_sz = 4096;
  flash.eraseSector(0);
#endif
 
  flash.waitUntilReady();
 
  // write all
  memset(bufwrite, (int) pattern, sizeof(bufwrite));
  Serial.printf("Write flash with 0x%02X\n", pattern);
  Serial.flush();
  ms = millis();
 
  for(uint32_t addr = 0; addr < flash_sz; addr += sizeof(bufwrite))
  {
    flash.writeBuffer(addr, bufwrite, sizeof(bufwrite));
  }
 
  uint32_t ms_write = millis() - ms;
  print_speed("Write ", flash_sz, ms_write);
  Serial.flush();
 
  // read and compare
  Serial.println("Read flash and compare");
  Serial.flush();
  uint32_t ms_read = 0;
  for(uint32_t addr = 0; addr < flash_sz; addr += sizeof(bufread))
  {
    memset(bufread, 0, sizeof(bufread));
 
    ms = millis();
    flash.readBuffer(addr, bufread, sizeof(bufread));
    ms_read += millis() - ms;
 
    if ( memcmp(bufwrite, bufread, BUFSIZE) )
    {
      Serial.printf("Error: flash contents mismatched at address 0x%08X!!!", addr);
      for(uint32_t i=0; i<sizeof(bufread); i++)
      {
        if ( i != 0 ) Serial.print(' ');
        if ( (i%16 == 0) )
        {
          Serial.println();
          Serial.printf("%03X: ", i);
        }
 
        Serial.printf("%02X", bufread[i]);
      }
 
      Serial.println();
      return false;
    }
  }
 
  print_speed("Read  ", flash_sz, ms_read);
  Serial.flush();
 
  return true;
}
 
void loop()
{
  // nothing to do
}
