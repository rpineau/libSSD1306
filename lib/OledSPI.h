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

#ifndef OLED_SPI_H
#define OLED_SPI_H

//------------------------------------------------------------------------
#include <sys/mman.h>
#include <array>
#include <cstdint>
#include <string>

#include "FileDescriptor.h"
#include "OledBitmap.h"
#include "OledHardware.h"
#include "OledPixel.h"
#include "point.h"

//------------------------------------------------------------------------
#define GPIO_REG_MAP            0xFF634000
#define GPIOX_FSEL_REG_OFFSET   0x116
#define GPIOX_OUTP_REG_OFFSET   0x117
#define GPIOX_INP_REG_OFFSET    0x118
#define GPIOA_FSEL_REG_OFFSET   0x120
#define GPIOA_OUTP_REG_OFFSET   0x121
#define GPIOA_INP_REG_OFFSET    0x122
#define BLOCK_SIZE              (4*1024)

namespace SSD1306
{
//------------------------------------------------------------------------

class OledSPI : public OledHardware, public OledPixel
{
public:

    static constexpr int Width{128};
    static constexpr int Height{64};
    static constexpr int BytesPerBlock{32};
    static constexpr int BufferSize{BytesPerBlock + 1};
    static constexpr int Blocks{(Width * Height) / (8 * BytesPerBlock)};
    static constexpr int ColumnsPerBlock{BytesPerBlock};
    static constexpr int ColumnsPerRow{Width / ColumnsPerBlock};
    static constexpr int DataOffset{1};

    OledSPI(const std::string& device, bool bIsSSH1106 = false);

    virtual ~OledSPI();

    OledSPI(const OledSPI&) = delete;
    OledSPI& operator= (const OledSPI&) = delete;

    void clear() override;
    void fill() override;
    bool isSetPixel(SSD1306::OledPoint p) const override;
    void setPixel(SSD1306::OledPoint p) override;
    void unsetPixel(SSD1306::OledPoint p) override;
    void xorPixel(SSD1306::OledPoint p) override;

    int width() const override { return Width; }
    int height() const override { return Height; }

    OledBitmap<Width, Height> getBitmap() const;

    void displayInverse() const override;
    void displayNormal() const override;
    void displayOff() const override;
    void displayOn() const override;
    void displaySetContrast(uint8_t contrast) const override;
    void displayUpdate() override;

private:

    void fillWith(uint8_t value);
    void init() const;
    void sendCommand(uint8_t command) const;
    void sendCommand(uint8_t command, uint8_t value) const;
    void sendCommand(uint8_t command, uint8_t v1, uint8_t v2) const;
    void sendData(uint8_t *data, int len) const;

    void commandMode(void) const;
    void dataMode(void) const;

    FileDescriptor fd_;
    
    struct PixelBlock
    {
        PixelBlock();

        std::array<uint8_t, BufferSize> bytes_;
        bool dirty_;
    };

    std::array<PixelBlock, Blocks> blocks_;
    uint8_t c_offset;
    // SPI stuff
    uint8_t mode;
    uint8_t bits;
    uint32_t speed;
    uint16_t delay;
    // pin access via memory mapped IO
    volatile uint32_t *gpio;
};

//------------------------------------------------------------------------

} // namespace SSD1306

//------------------------------------------------------------------------

#endif
