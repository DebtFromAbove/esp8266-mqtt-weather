#ifndef ESPLogging_h
#define ESPLogging_h

#include "Arduino.h"
typedef struct {
  uint8_t msgLevel;
  String message;
} logMsg;

class ESPLogging {
  public:
    ESPLogging();
    void begin(uint8_t level, uint8_t maxLogs, HardwareSerial* serial=NULL);
    void stopLogging();
    void startLogging();
    bool error(String msg);
    bool warn(String msg);
    bool info(String msg);
    bool debug(String msg);
    uint8_t numLogs();
    uint8_t getLogs(String* msgs);
    uint8_t getLevels(uint8_t* lvls);
  private:
    uint8_t _maxLogs;
    uint8_t _loglevel;
    uint8_t _position;
    uint8_t _numMessages;
    bool _allowLogging;
    logMsg* _messages;
    void _addLog(String msg, uint8_t lvl);
    String _levelText(uint8_t level);
    HardwareSerial* _HardSerial;

};

#endif
