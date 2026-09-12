#include "Protocol.h"

void sendFrame(uint8_t func, const uint8_t *data, uint8_t n) {
  uint8_t len = n + 3;                       // LEN + FUNC + data + CHK, -1 (= total-2)
  uint8_t sum = len + func;
  for (uint8_t i = 0; i < n; i++) sum += data[i];
  Serial.write(PTO_HEAD);
  Serial.write(PTO_ID_TX);
  Serial.write(len);
  Serial.write(func);
  if (n) Serial.write(data, n);
  Serial.write((uint8_t)(sum & 0xFF));
}

void FrameParser::feed(uint8_t b) {
  if (_len == 0) { if (b == PTO_HEAD) _buf[_len++] = b; return; }
  if (_len == 1) { if (b == PTO_ID_RX) _buf[_len++] = b; else _len = 0; return; }
  _buf[_len++] = b;
  if (_len >= 3) {
    uint8_t total = _buf[2] + 2;
    if (total < 5 || total > sizeof(_buf)) { _len = 0; return; }
    if (_len == total) {
      uint8_t sum = 0;
      for (uint8_t i = 2; i < total - 1; i++) sum += _buf[i];
      if (sum == _buf[total - 1] && _h)
        _h(_buf[3], &_buf[4], total - 5);
      _len = 0;
    }
  }
}
