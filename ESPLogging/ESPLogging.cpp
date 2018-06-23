#include "Arduino.h"
#include "ESPLogging.h"

ESPLogging::ESPLogging() {
  _numMessages = 0;
  _position = 0;
  _allowLogging = true;
}

void ESPLogging::begin(uint8_t level, uint8_t maxLogs, HardwareSerial* serial) {
    _maxLogs = maxLogs;
    _loglevel = level;
    _messages = new logMsg[maxLogs];
    _HardSerial = serial;
}

void ESPLogging::stopLogging() {
  _allowLogging = false;
}

void ESPLogging::startLogging() {
  _allowLogging = true;
}

void ESPLogging::_addLog(String msg, uint8_t lvl) {
  _numMessages += 1;
  _messages[_position].message = msg;
  _messages[_position].msgLevel = lvl;
  _position += 1;
  if (_position >= _maxLogs) {
    _position = 0;
  }

  if ( _HardSerial) {
    _HardSerial->println(_levelText(lvl) + ": " + msg);
  }
}

bool ESPLogging::error(String msg) {
  if ((_loglevel >= 1) && _allowLogging) {
    _addLog(msg, 1);
    return true;
  }
  else {
    // Logging is disabled (0), exit with error
    return false;
  }
}

bool ESPLogging::warn(String msg) {
  if ((_loglevel >= 2) && _allowLogging) {
    _addLog(msg, 2);
    return true;
  }
  else {
    // Logging is disabled or ERROR or below, exit with error
    return false;
  }
}

bool ESPLogging::info(String msg) {
  if ((_loglevel >= 3) && _allowLogging) {
    _addLog(msg, 3);
    return true;
  }
  else {
    // Logging is disabled or WARN or below, exit with error
    return false;
  }
}

bool ESPLogging::debug(String msg) {
  if ((_loglevel >= 4) && _allowLogging) {
    _addLog(msg, 4);
    return true;
  }
  else {
    // Logging is disabled or INFO or below, exit with error
    return false;
  }
}

uint8_t ESPLogging::numLogs() {
  return _numMessages;
}

uint8_t ESPLogging::getLevels(uint8_t* lvls) {
  if (_numMessages == 0) {
    return 0;
  }
  if (_numMessages > _maxLogs) {
    uint8_t ptr = _position + 1;
    for (uint8_t i=0; i<_maxLogs; i++) {
      if (ptr >= _maxLogs) {
        ptr = 0;
      }
      lvls[i] = _messages[ptr].msgLevel;
      ptr += 1;
    }
    return _maxLogs;
  }
  else {
    // easy case, just loop till end
    for (uint8_t i=0; i<_numMessages; i++) {
      lvls[i] = _messages[i].msgLevel;
    }
    return _numMessages;
  }

}

uint8_t ESPLogging::getLogs(String* msgs) {
  if (_numMessages == 0) {
    return 0;
  }
  if (_numMessages > _maxLogs) {
    //slightly trickier case
    uint8_t ptr = _position + 1;
    for (uint8_t i=0; i<_maxLogs; i++) {
      if (ptr >= _maxLogs) {
        ptr = 0;
      }
      Serial.println(String(_messages[ptr].msgLevel));
      Serial.println(String(_levelText(_messages[ptr].msgLevel)));

      String ml = _levelText(_messages[ptr].msgLevel);
      msgs[i] = ml + String(": ") + _messages[ptr].message;
      ptr += 1;
    }
    return _maxLogs;
  }
  else {
    // easy case, just loop till end
    for (uint8_t i=0; i<_numMessages; i++) {
      String ml = _levelText(_messages[i].msgLevel);
      msgs[i] = ml + String(": ") + _messages[i].message;
    }
    return _numMessages;
  }
}

String ESPLogging::_levelText(uint8_t level) {
  switch(level) {
    case 1:
      return "ERROR";
      break;
    case 2:
      return "WARN";
      break;
    case 3:
      return "INFO";
      break;
    case 4:
      return "DEBUG";
      break;
    default:
      return "DISABLED";
      break;
  }
}
