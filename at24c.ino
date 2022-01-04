#include <Wire.h>

// pagesize =   8: AT24C01, AT24C02
// pagesize =  16: AT24C04, AT24C08, AT24C16
// pagesize =  32: AT24C32, AT24C64
// pagesize =  64: AT24C128, AT24C256
// pagesize = 128: AT24C512, AT24C1024, AT24C1025

// sz: is_ACK(loc + sz)         for 02, 04, 08, 16
// sz: memcmp(loc + sz, loc, n) for 01, 32, 64, 128, 256, 512

char printbuf[80];
#define PRINTF(fmt ...)  { sprintf(printbuf, fmt); Serial.print(printbuf); }
#define address_ACK(slave) write_I2C_buf(slave, NULL, 0)

bool is_24Cxxx = true;   //true for 24C32 - 24C1025
bool is_AT24CS = false;  //true for AT24CS01 - AT24CS128

bool write_I2C_buf(uint8_t slave, uint8_t *buf, int n)
{
    Wire.beginTransmission(slave);
    while (n--) {    //Wire defaults to max 32 bytes
        Wire.write(*buf++);
    }
    return Wire.endTransmission() == 0; //true if good
}

void read_I2C_buf(uint8_t slave, uint8_t *buf, int n)
{
    uint8_t ask;
    while (n) {      //read arbitrary amount (in chunks)
        ask = (n > 16) ? 16 : n;
        n -= ask;
        Wire.requestFrom(slave, ask); // request 16 bytes from slave device #2
        while (Wire.available() && ask--) { // slave may send less than requested
            *buf++ = Wire.read();      // receive a byte as character
        }
    }
}

bool check_24Cxxx(uint8_t slave)
{
    uint8_t wbuf[2] = {0, 0};
    write_I2C_buf(slave, &wbuf[0], 1); //sets location on 1-byte chips
    read_I2C_buf(slave, &wbuf[1], 1);  //reads the first byte
    write_I2C_buf(slave, wbuf, 2); //invokes page-write on 24C01
    bool ret = address_ACK(slave); //24C32 will ACK, 24C01 will NAK
    if (!ret) delay(10); //wait for 24C01 page-write to complete
    return ret;
}

bool write_24Cxxx_buf(uint8_t slave, uint16_t loc, uint8_t *buf, int n)
{
    uint8_t wbuf[n + 2], *p = wbuf;
    if (is_24Cxxx) *p++ = loc >> 8;
    else slave |= ((loc >> 8) & 7);
    *p++ = loc & 0xFF;
    for (int i = 0; i < n; i++) *p++ = buf[i];
    return write_I2C_buf(slave, wbuf, n + 1 + is_24Cxxx);
}

bool read_24Cxxx_buf(uint8_t slave, uint16_t loc, uint8_t *buf, int n)
{
    bool ret = write_24Cxxx_buf(slave, loc, NULL, 0);
    if (!is_24Cxxx) slave |= ((loc >> 8) & 7);
    if (ret) read_I2C_buf(slave, buf, n);
    return ret;
}

void hexdump(uint16_t loc, uint8_t *block, int16_t n)
{
    int16_t cnt;
    char ascbuf[17], *p, wid = 16;
    while (n > 0) {
        PRINTF("%04X:", loc);
        p = ascbuf;
        cnt = n;
        if (cnt > wid) cnt = wid;
        loc += cnt;
        n -= cnt;
        while (cnt--) {
            uint8_t c = *block++;
            *p++ = (c >= ' ' && c < 128) ? c : '.';
            PRINTF(" %02X", c);
        }
        *p = 0;
        PRINTF(" *%s*\r\n", ascbuf);
    }
}

void dump_AT24(uint8_t slave, uint16_t loc, int32_t sz)
{
    uint8_t pagesize = 16, buf[pagesize];
    for (uint32_t ads = loc; sz > 0; ads += pagesize, sz -= pagesize) {
        read_24Cxxx_buf(slave, ads, buf, pagesize);
        hexdump(ads, buf, pagesize);
    }
}

void erase_AT24(uint8_t slave, uint16_t loc, int32_t sz, uint8_t pagesize)
{
    uint8_t buf[pagesize];
    memset(buf, 0xFF, sizeof(buf));
    for (int32_t ads = loc; sz > 0; ads += pagesize, sz -= pagesize) {
        write_24Cxxx_buf(slave, ads, buf, pagesize);
        delay(10);
    }
}

void test_AT24xxx(uint8_t slave)
{
    uint16_t loc = 0x80 - 8;
    uint16_t page = loc & ~127;
    uint8_t buf[128];
    uint8_t msg[] = "page boundary cross";
    int msglen = strlen((char*)msg);
    int pagesize = 8;
    //erase_AT24(slave, 0, 2048, pagesize);
    write_24Cxxx_buf(slave, loc, msg, msglen);
    delay(10);   // typical AT24C01 is 5ms
    read_24Cxxx_buf(slave, page, buf, 128);
    hexdump(page, buf, 128);
    for (pagesize = 8; pagesize <= 128; pagesize <<= 1) {
        if (pagesize == 8 && memcmp(buf + 128 - pagesize, "ossry cr", 8) == 0) break;
        else if (memcmp(buf + 128 - pagesize, "ndary cross", 11) == 0) break;
    }
    if (pagesize > 128) pagesize = -1;
    uint8_t mirror[8];
    uint32_t sz;
    for (sz = 128; sz <= 65536 ; sz <<= 1) {
        bool ret = read_24Cxxx_buf(slave, loc + sz, mirror, 8);
        if (ret == false) break;
        if (memcmp(mirror, buf + loc - page, 8) == 0) break;
    }
    PRINTF("page size = %d mirror size = %ld e.g. AT24C%s%02d\r\n",
           pagesize, sz, is_AT24CS ? "S" : "", (int)(sz / 128));
    if (is_AT24CS) {
        dump_AT24(slave + 8, is_24Cxxx ? 0x0800 : 0x80, 16);
    }
    //dump_AT24(slave, 0, 1024); // i.e 24C08
}

void setup()
{
    Serial.begin(9600);
    Wire.begin();        // join i2c bus (address optional for master)
    PRINTF("diagnose 24Cxx devices\r\n");
    for (uint8_t slave = 0x50; slave < 0x58; slave++) {
        bool found = address_ACK(slave);
        delay(1);
        if (found) {
            is_24Cxxx = check_24Cxxx(slave);
            is_AT24CS = address_ACK(slave + 0x08);
            PRINTF("I2C Device found @ 0x%02X %s%s\r\n",
                   slave, is_24Cxxx ? " 2-byte loc" : " AT24C01.16",
                   is_AT24CS ? " with SerialNo" : "");
            if (!is_24Cxxx && slave > 0x50 && slave < 0x54)
                continue; //skip 24C08 extras
            test_AT24xxx(slave);
        }
    }
}

void loop()
{
}
