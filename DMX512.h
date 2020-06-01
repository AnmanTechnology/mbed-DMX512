#ifndef _DMX512_H_
#define _DMX512_H_

#include <mbed.h>
#define C_INIT (uint8_t *)0x801F000
#define C_CLR 0x20
#define DMX_SIZE 193
#define DMX_START_CODE 0

#define XMIT_TX 1
#define XMIT_RX 0

#define DMX_TIME_BREAK 100 // 100us (88us-1s)
#define DMX_TIME_MAB 12    // 12us (8us-1s)
#define DMX_TIME_MBB 200   // 10us (0-1s)

enum DMX_MODE
{
    DMX_MODE_BEGIN,
    DMX_MODE_START,
    DMX_MODE_BREAK,
    DMX_MODE_MAB,
    DMX_MODE_DATA,
    DMX_MODE_ERROR,
    DMX_MODE_STOP,
};

class DMX512
{
public:
    /** init DMX class
     * @param p_tx TX serial port 
     * @param p_rx RX serial port 
     * @param p_re data enable/~receive enable
     */
    DMX512(PinName p_tx, PinName p_rx, PinName p_xmit = NC);
    /** Send the data
     * @param addr DMX data address (0-511)
     * @param data DMX data (0-255)
     */
    void put(int addr, int data);
    /** Send the data
     * @param buf DMX data buffer
     * @param addr DMX data address
     * @param len data length
     */
    // void put(unsigned char *buf, int addr = 0, int len = DMX_SIZE);
    /** Get the data
     * @param addr DMX data address (0-511)
     * @return DMX data (0-255)
     */
    int get(int addr);
    /** Get the data
     * @param buf DMX data buffer
     * @param addr DMX data address
     * @param len data length
     */
    // void get(unsigned char *buf, int addr = 0, int len = DMX_SIZE);
    /** Start DMX send operation
     */
    void start();
    /** Stop DMX send operation
     */
    void stop();
    /** Clear DMX data
     */
    void clear();
    int isReceived();
    int isSent();
    // unsigned char *getRxBuffer();
    // unsigned char *getTxBuffer();
    // int setTimingParameters(int breaktime, int mab, int mbb);

private:
    void int_timer();
    void int_tx();
    void int_rx();
    RawSerial _dmx;
    Timeout timeout01;
    DigitalOut *_xmit;

    volatile DMX_MODE mode_tx, mode_rx;
    volatile int addr_tx, addr_rx;

    uint8_t data_tx[DMX_SIZE];
    uint8_t data_rx[DMX_SIZE];
    uint8_t data_rx_working[DMX_SIZE];

    volatile int is_received, is_sent;
    int time_break, time_mab, time_mbb;
    Timer t;

    USART_TypeDef *_uart;
};

DMX512::DMX512(PinName p_tx, PinName p_rx, PinName p_xmit) : _dmx(p_tx, p_rx, 250000)
{
    if (p_xmit == NC)
    {
        _xmit = NULL;
    }
    else
    {
        _xmit = new DigitalOut(p_xmit, XMIT_RX);
    }
    clear();
    mode_tx = DMX_MODE_STOP;
    mode_rx = DMX_MODE_BEGIN;
    is_received = 0;
    is_sent = 0;

    time_break = DMX_TIME_BREAK;
    time_mab = DMX_TIME_MAB;
    time_mbb = DMX_TIME_MBB;

    _uart = (USART_TypeDef *)USART2;
    // NVIC_SetPriority(USART2_IRQn, 1);

    _dmx.format(8, Serial::None, 2);
    _dmx.attach(callback(this, &DMX512::int_rx), Serial::RxIrq);
    t.start();
}

void DMX512::put(int addr, int data)
{
    if (addr < 0 || addr >= DMX_SIZE)
        return;
    data_tx[addr] = data;
}

int DMX512::get(int addr)
{
    if (addr < 0 || addr >= DMX_SIZE || addr & C_CLR)
        return -1;
    return data_rx[addr - 1];
}

void DMX512::int_timer()
{
    switch (mode_tx)
    {
    case DMX_MODE_BEGIN:
        timeout01.detach();
        mode_tx = DMX_MODE_BREAK;
        timeout01.attach_us(callback(this, &DMX512::int_timer), time_break);
        break;
    case DMX_MODE_BREAK:
        timeout01.detach();
        mode_tx = DMX_MODE_MAB;
        timeout01.attach_us(callback(this, &DMX512::int_timer), time_mab);
        break;
    case DMX_MODE_MAB:
        timeout01.detach();
        addr_tx = 0;
        mode_tx = DMX_MODE_DATA;

        _dmx.attach(callback(this, &DMX512::int_tx), Serial::TxIrq);
        _dmx.putc(DMX_START_CODE);
        break;
    default:
        break;
    }
}

void DMX512::int_tx()
{
    if (mode_tx == DMX_MODE_DATA)
    {
        if (addr_tx < DMX_SIZE)
        {
            _dmx.putc(data_tx[addr_tx]);
            addr_tx++;
        }
        else
        {
            _dmx.attach(0, Serial::TxIrq);
            mode_tx = DMX_MODE_BEGIN;
            is_sent = 1;
            timeout01.attach_us(callback(this, &DMX512::int_timer), time_mbb);
        }
    }
}

void DMX512::int_rx()
{
    char dat;

    int tim = t.read_us();
    t.reset();
    dat = _dmx.getc();

    if (tim >= 700)
    {
        if (addr_rx >= 24 && mode_rx == DMX_MODE_DATA)
        {
            memcpy(data_rx, data_rx_working, addr_rx);
            is_received = 1;
        }
        mode_rx = DMX_MODE_BREAK;
        return;
    }

    if (mode_rx == DMX_MODE_BREAK)
    {
        if (dat == DMX_START_CODE)
        {
            addr_rx = 0;
            mode_rx = DMX_MODE_DATA;
        }
        else
        {
            mode_rx = DMX_MODE_ERROR;
        }
    }

    else if (mode_rx == DMX_MODE_DATA)
    {

        data_rx_working[addr_rx] = dat;
        addr_rx++;
        if (addr_rx >= DMX_SIZE)
        {
            memcpy(data_rx, data_rx_working, sizeof(data_rx_working));
            is_received = 1;
            mode_rx = DMX_MODE_BEGIN;
        }
    }
}

void DMX512::start()
{
    if (mode_tx == DMX_MODE_STOP)
    {
        if (_xmit)
            _xmit->write(XMIT_TX);
        mode_tx = DMX_MODE_BEGIN;
        is_sent = 0;
        timeout01.attach_us(callback(this, &DMX512::int_timer), time_mbb);
    }
}

void DMX512::stop()
{
    _dmx.attach(0, Serial::TxIrq);
    timeout01.detach();
    mode_tx = DMX_MODE_STOP;
    if (_xmit)
        _xmit->write(XMIT_RX);
}

void DMX512::clear()
{
    memset(data_tx, 0, sizeof(data_tx));
    memset(data_rx, 0, sizeof(data_rx));
    memset(data_rx_working, 0, sizeof(data_rx_working));
}

int DMX512::isReceived()
{
    int r = is_received;
    is_received = 0;
    return r;
}

int DMX512::isSent()
{
    int r = is_sent;
    is_sent = 0;
    return r;
}

#endif