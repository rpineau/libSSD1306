//-------------------------------------------------------------------------
//
// The MIT License (MIT)
//
// Copyright (c) 2017 Andrew Duncan
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
// CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//-------------------------------------------------------------------------

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

#include <algorithm>
#include <system_error>

#include <linux/spi/spidev.h>
#include "OledSPI.h"

//------------------------------------------------------------------------

namespace
{

    // address modes

    constexpr uint8_t OLED_HORIZONTAL_ADDRESSING_MODE{0x00};
    constexpr uint8_t OLED_VERTICAL_ADDRESSING_MODE{0x01};
    constexpr uint8_t OLED_PAGE_ADDRESSING_MODE{0x02};

    // commands

    constexpr uint8_t OLED_SET_COLUMN_START_LOW_MASK{0x00};
    constexpr uint8_t OLED_SET_COLUMN_START_HIGH_MASK{0x10};
    constexpr uint8_t OLED_SET_MEMORY_ADDRESSING_MODE{0x20};
    constexpr uint8_t OLED_SET_COLUMN_ADDRESS{0x21};
    constexpr uint8_t OLED_SET_PAGE_ADDRESS{0x22};
    constexpr uint8_t OLED_SET_DISPLAY_START_LINE_MASK{0x40};
    constexpr uint8_t OLED_SET_CONTRAST{0x81};
    constexpr uint8_t OLED_ENABLE_CHARGE_PUMP_REGULATOR{0x8D};
    constexpr uint8_t OLED_SET_SEGMENT_REMAP_0{0xA0};
    constexpr uint8_t OLED_SET_SEGMENT_REMAP_127{0xA1};
    constexpr uint8_t OLED_SET_ENTIRE_DISPLAY_ON_RESUME{0xA4};
    constexpr uint8_t OLED_SET_ENTIRE_DISPLAY_ON_FORCE{0xA5};
    constexpr uint8_t OLED_SET_NORMAL_DISPLAY{0xA6};
    constexpr uint8_t OLED_SET_INVERSE_DISPLAY{0xA7};
    constexpr uint8_t OLED_SET_MUX_RATIO{0xA8};
    constexpr uint8_t OLED_SET_DISPLAY_OFF{0xAE};
    constexpr uint8_t OLED_SET_DISPLAY_ON{0xAF};
    constexpr uint8_t OLED_SET_PAGE_START_ADDRESS_MASK{0xB0};
    constexpr uint8_t OLED_SET_COM_OUTPUT_SCAN_DIRECTION_NORMAL{0xC0};
    constexpr uint8_t OLED_SET_COM_OUTPUT_SCAN_DIRECTION_REMAP{0xC8};
    constexpr uint8_t OLED_SET_DISPLAY_OFFSET{0xD3};
    constexpr uint8_t OLED_SET_OSC_FREQUENCY{0xD5};
    constexpr uint8_t OLED_SET_PRECHARGE_PERIOD{0xD9};
    constexpr uint8_t OLED_SET_COM_PINS_HARDWARE_CONFIGURATION{0xDA};
    constexpr uint8_t OLED_SET_VCOMH_DESELECT_LEVEL{0xDB};

    //--------------------------------------------------------------------

    struct PixelOffset
    {
        PixelOffset(SSD1306::OledPoint p)
        :
            bit{p.y() % 8},
            block{(p.x() / SSD1306::OledSPI::ColumnsPerBlock)
                  + (SSD1306::OledSPI::ColumnsPerRow * (p.y() / 8))},
            byte{SSD1306::OledSPI::DataOffset
                 + (p.x() % SSD1306::OledSPI::ColumnsPerBlock)}
        {
        }

        int bit;
        int block;
        int byte;
    };
}

//------------------------------------------------------------------------

SSD1306::OledSPI::OledSPI(
    const std::string& device,
    bool bIsSSH1106)
:
    fd_{-1},
    blocks_{}
{
    int fd;
    bits = 8;
    speed = 8000000;
    mode = SPI_MODE_0;
    delay = 0;

    if ((fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0) {
        printf("Unable to open /dev/gpiomem\n");
        return ;
    }

    gpio = (uint32_t*)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, GPIO_REG_MAP);
    // Set direction of GPIOA.4 register to out
    *(gpio + (GPIOA_FSEL_REG_OFFSET)) &= ~(1 << 4);

    if (gpio < 0) {
        printf("Mmap failed.\n");
        return ;
    }

    if(bIsSSH1106)
        c_offset = 2;
    else
        c_offset = 0;
    fd_ = FileDescriptor{::open(device.c_str(), O_RDWR)};

    if (fd_.fd() == -1)
    {
        std::string what( "open "
                        + device
                        + " " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }

    // SPI mode
    if (ioctl(fd_.fd(), SPI_IOC_WR_MODE, &mode) == -1)
    {
        std::string what( "ioctl SPI_IOC_WR_MODE " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }


    // bits per word
    if (ioctl(fd_.fd(), SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
    {
        std::string what( "ioctl SPI_IOC_WR_BITS_PER_WORD " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }

    // SPI speed
    if (ioctl(fd_.fd(), SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
    {
        std::string what( "ioctl SPI_IOC_WR_MAX_SPEED_HZ " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }


    init();
}

//------------------------------------------------------------------------

SSD1306::OledSPI::~OledSPI() = default;

//------------------------------------------------------------------------

SSD1306::OledSPI::PixelBlock::PixelBlock()
:
    bytes_{0x00},
    dirty_{true}
{
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::clear()
{
    fillWith(0x00);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::fill()
{
    fillWith(0xFF);
}

//------------------------------------------------------------------------

bool
SSD1306::OledSPI::isSetPixel(
    SSD1306::OledPoint p) const
{
    if (not pixelInside(p))
    {
        return false;
    }

    PixelOffset po{p};

    return blocks_[po.block].bytes_[po.byte] & (1 << po.bit);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::setPixel(
    SSD1306::OledPoint p)
{
    if (not pixelInside(p))
    {
        return;
    }

    PixelOffset po{p};

    if ((blocks_[po.block].bytes_[po.byte] & (1 << po.bit)) == 0)
    {
        blocks_[po.block].bytes_[po.byte] |= (1 << po.bit);

        if (not blocks_[po.block].dirty_)
        {
            blocks_[po.block].dirty_ = true;
        }
    }
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::unsetPixel(
    SSD1306::OledPoint p)
{
    if (not pixelInside(p))
    {
        return;
    }

    PixelOffset po{p};

    if ((blocks_[po.block].bytes_[po.byte] & (1 << po.bit)) != 0)
    {
        blocks_[po.block].bytes_[po.byte] &= ~(1 << po.bit);

        if (not blocks_[po.block].dirty_)
        {
            blocks_[po.block].dirty_ = true;
        }
    }
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::xorPixel(
    SSD1306::OledPoint p)
{
    if (not pixelInside(p))
    {
        return;
    }

    PixelOffset po{p};

    if ((blocks_[po.block].bytes_[po.byte] & (1 << po.bit)) == 0)
    {
        blocks_[po.block].bytes_[po.byte] |= (1 << po.bit);
    }
    else
    {
        blocks_[po.block].bytes_[po.byte] &= ~(1 << po.bit);
    }

    if (not blocks_[po.block].dirty_)
    {
        blocks_[po.block].dirty_ = true;
    }
}

//------------------------------------------------------------------------

SSD1306::OledBitmap<SSD1306::OledSPI::Width, SSD1306::OledSPI::Height>
SSD1306::OledSPI::getBitmap() const
{
    OledBitmap<Width, Height> bitmap;

    for (auto y = 0 ; y < Height ; ++y)
    {
        for (auto x = 0 ; x < Width ; ++x)
        {
            OledPoint p{x, y};

            if (isSetPixel(p))
            {
                bitmap.setPixel(p);
            }
        }
    }

    return bitmap;
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::displayInverse() const
{
    sendCommand(OLED_SET_INVERSE_DISPLAY);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::displayNormal() const
{
    sendCommand(OLED_SET_NORMAL_DISPLAY);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::displayOff() const
{
    sendCommand(OLED_SET_DISPLAY_OFF);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::displayOn() const
{
    sendCommand(OLED_SET_DISPLAY_ON);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::displaySetContrast(
    uint8_t contrast) const
{
    sendCommand(OLED_SET_CONTRAST);
    sendCommand(contrast);
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::displayUpdate()
{
    uint8_t page{0};
    uint8_t column{0};

    for (auto& block : blocks_)
    {
        if (block.dirty_)
        {
            uint8_t column_low = column & 0xF;
            uint8_t column_high = (column >> 4) & 0x0F;

            sendCommand(OLED_SET_PAGE_START_ADDRESS_MASK | page);
            sendCommand(OLED_SET_COLUMN_START_LOW_MASK | column_low | c_offset);
            sendCommand(OLED_SET_COLUMN_START_HIGH_MASK | column_high);
            sendData(block.bytes_.data(), block.bytes_.size());
            block.dirty_ = false;
        }

        column += OledSPI::ColumnsPerBlock;

        if (column >= Width)
        {
            column = 0;
            page += 1;
        }
    }
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::fillWith(
    uint8_t value)
{
    for (auto& block : blocks_)
    {
        auto& bytes = block.bytes_;

        for (auto byte = bytes.begin() + SSD1306::OledSPI::DataOffset ;
             byte != bytes.end() ;
             ++byte)
        {
            if (*byte != value)
            {
                *byte = value;

                if (not block.dirty_)
                {
                    block.dirty_ = true;
                }
            }
        }
    }
}

//------------------------------------------------------------------------

void
SSD1306::OledSPI::init() const
{
    // Enable charge pump regulator - 8Dh, 14h

    sendCommand(OLED_ENABLE_CHARGE_PUMP_REGULATOR, 0x14);

    // Set Memory Addressing Mode - 20h, 02h

    sendCommand(OLED_SET_MEMORY_ADDRESSING_MODE, OLED_PAGE_ADDRESSING_MODE);

    // Set Osc Frequency D5h, 80h

    sendCommand(OLED_SET_OSC_FREQUENCY, 0x80);

    // Set Display Offset - D3h, 00h

    sendCommand(OLED_SET_DISPLAY_OFFSET, 0x00);

    // Set Display Start Line - 40h

    sendCommand(OLED_SET_DISPLAY_START_LINE_MASK | 0x00);

    // Set Segment re-map - A0h/A1h

    sendCommand(OLED_SET_SEGMENT_REMAP_127);

    // Set COM Output Scan Direction - C0, C8h

    sendCommand(OLED_SET_COM_OUTPUT_SCAN_DIRECTION_REMAP);

    // Set COM Pins hardware configuration - DAh, 12h

    sendCommand(OLED_SET_COM_PINS_HARDWARE_CONFIGURATION, 0x12);

    // Set Pre-charge Period D9h, F1h

    sendCommand(OLED_SET_PRECHARGE_PERIOD, 0xF1);

    // Set Vcomh Deselect Level - DBh, 40h

    sendCommand(OLED_SET_VCOMH_DESELECT_LEVEL, 0x40);

    // Disable Entire Display On - A4h

    sendCommand(OLED_SET_ENTIRE_DISPLAY_ON_RESUME);

    // Set Normal Display - A6h

    sendCommand(OLED_SET_NORMAL_DISPLAY);

    // Set Column Address - 21h, 00h, 7Fh

    sendCommand(OLED_SET_COLUMN_ADDRESS, 0x00, 0x7F);

    // Set Page Address - 22h, 00h, 07h

    sendCommand(OLED_SET_PAGE_ADDRESS, 0x00, 0x07);

    // Set Contrast Control - 81h, 7Fh

    sendCommand(OLED_SET_CONTRAST, 0x7F);

    // Display On - AFh

    sendCommand(OLED_SET_DISPLAY_ON);

    usleep(100000);
}

//------------------------------------------------------------------------

void SSD1306::OledSPI::sendCommand(uint8_t command) const
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    commandMode();

    std::array<uint8_t, 1> data{command};

    tr.tx_buf = (unsigned long)data.data();
    tr.rx_buf = 0;
    tr.len = data.size();
    tr.delay_usecs = delay;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.cs_change = 0;


   if (ioctl(fd_.fd(), SPI_IOC_MESSAGE(1), &tr) < 1)
    {
        std::string what( "write " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }
}

//------------------------------------------------------------------------

void SSD1306::OledSPI::sendCommand(uint8_t command, uint8_t value) const
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    commandMode();
    std::array<uint8_t, 2> data{ command, value};

    tr.tx_buf = (unsigned long)data.data();
    tr.rx_buf = 0;
    tr.len = data.size();
    tr.delay_usecs = delay;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.cs_change = 0;


   if (ioctl(fd_.fd(), SPI_IOC_MESSAGE(1), &tr) < 1) {
        std::string what( "write " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }
}

//------------------------------------------------------------------------

void SSD1306::OledSPI::sendCommand(uint8_t command, uint8_t v1, uint8_t v2) const
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    commandMode();
    std::array<uint8_t, 3> data{ command, v1, v2};

    tr.tx_buf = (unsigned long)data.data();
    tr.rx_buf = 0;
    tr.len = data.size();
    tr.delay_usecs = delay;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.cs_change = 0;

   if (ioctl(fd_.fd(), SPI_IOC_MESSAGE(1), &tr) < 1) {
        std::string what( "write " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }
}


void SSD1306::OledSPI::sendData(uint8_t *data, int len) const
{
    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));

    dataMode();
    tr.tx_buf = (unsigned long)data;
    tr.rx_buf = 0;
    tr.len = len;
    tr.delay_usecs = delay;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.cs_change = 0;

    if (ioctl(fd_.fd(), SPI_IOC_MESSAGE(1), &tr) < 1) {
        std::string what( "write " __FILE__ "("
                        + std::to_string(__LINE__)
                        + ")" );
        throw std::system_error(errno, std::system_category(), what);
    }

}


void SSD1306::OledSPI::commandMode() const
{
    // Set GPIOA.4 to low
    *(gpio + (GPIOA_OUTP_REG_OFFSET)) &= ~(1 << 4);
}

void SSD1306::OledSPI::dataMode() const
{
    // Set GPIOA.4 to high
    *(gpio + (GPIOA_OUTP_REG_OFFSET)) |= (1 << 4);
}


