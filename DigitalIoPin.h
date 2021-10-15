#ifndef IO_H_
#define IO_H_

void Sleep(int ms);

class DigitalIoPin {
public:
    DigitalIoPin(int port, int pin, bool input = true, bool pullup = true, bool invert = false);
    virtual ~DigitalIoPin() { };
    bool read();
    void write(bool on);
    void toggle();
private:
    int port;
    int pin;
    bool input;
    bool pullup;
    bool invert;
};

#endif