/*
This is the LoRa deserialization code for the payload decoder in The Things Network Console. You can use this to deserialize the received payload from the Feather M0.
The main part of this code has been copied from https://github.com/thesolarnomad/lora-serialization

The MIT License (MIT)

Copyright (c) 2017 Joscha Feth

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
function Decoder(bytes) {
  // bytes is of type Buffer

  return decode(
    bytes,
    [ latLng, humidity  ],
    [ 'coords', 'battery' ]
  );
}

var bytesToInt = function(bytes) {
  var i = 0;
  for (var x = 0; x < bytes.length; x++) {
    i |= +(bytes[x] << (x * 8));
  }
  return i;
};

var unixtime = function(bytes) {
  if (bytes.length !== unixtime.BYTES) {
    throw new Error('Unix time must have exactly 4 bytes');
  }
  return bytesToInt(bytes);
};
unixtime.BYTES = 4;

var uint8 = function(bytes) {
  if (bytes.length !== uint8.BYTES) {
    throw new Error('int must have exactly 1 byte');
  }
  return bytesToInt(bytes);
};
uint8.BYTES = 1;

var uint16 = function(bytes) {
  if (bytes.length !== uint16.BYTES) {
    throw new Error('int must have exactly 2 bytes');
  }
  return bytesToInt(bytes);
};
uint16.BYTES = 2;

var latLng = function(bytes) {
  if (bytes.length !== latLng.BYTES) {
    throw new Error('Lat/Long must have exactly 8 bytes');
  }

  var lat = bytesToInt(bytes.slice(0, latLng.BYTES / 2));
  var lng = bytesToInt(bytes.slice(latLng.BYTES / 2, latLng.BYTES));

  return [lat / 1e6, lng / 1e6];
};
latLng.BYTES = 8;

var temperature = function(bytes) {
  if (bytes.length !== temperature.BYTES) {
    throw new Error('Temperature must have exactly 2 bytes');
  }
  var isNegative = bytes[0] & 0x80;
  var b = ('00000000' + Number(bytes[0]).toString(2)).slice(-8)
        + ('00000000' + Number(bytes[1]).toString(2)).slice(-8);
  if (isNegative) {
    var arr = b.split('').map(function(x) { return !Number(x); });
    for (var i = arr.length - 1; i > 0; i--) {
      arr[i] = !arr[i];
      if (arr[i]) {
        break;
      }
    }
    b = arr.map(Number).join('');
  }
  var t = parseInt(b, 2);
  if (isNegative) {
    t = -t;
  }
  return t / 1e2;
};
temperature.BYTES = 2;

var humidity = function(bytes) {
  if (bytes.length !== humidity.BYTES) {
    throw new Error('Humidity must have exactly 2 bytes');
  }

  var h = bytesToInt(bytes);
  return h / 1e2;
};
humidity.BYTES = 2;

// Based on https://stackoverflow.com/a/37471538 by Ilya Bursov
// quoted by Arjan here https://www.thethingsnetwork.org/forum/t/decode-float-sent-by-lopy-as-node/8757
function rawfloat(bytes) {
  if (bytes.length !== rawfloat.BYTES) {
    throw new Error('Float must have exactly 4 bytes');
  }
  // JavaScript bitwise operators yield a 32 bits integer, not a float.
  // Assume LSB (least significant byte first).
  var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
  var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
  var e = bits>>>23 & 0xff;
  var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
  var f = sign * m * Math.pow(2, e - 150);
  return f;
}
rawfloat.BYTES = 4;

var bitmap = function(byte) {
  if (byte.length !== bitmap.BYTES) {
    throw new Error('Bitmap must have exactly 1 byte');
  }
  var i = bytesToInt(byte);
  var bm = ('00000000' + Number(i).toString(2)).substr(-8).split('').map(Number).map(Boolean);
  return ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    .reduce(function(obj, pos, index) {
      obj[pos] = bm[index];
      return obj;
    }, {});
};
bitmap.BYTES = 1;

var decode = function(bytes, mask, names) {

  var maskLength = mask.reduce(function(prev, cur) {
    return prev + cur.BYTES;
  }, 0);
  if (bytes.length < maskLength) {
    throw new Error('Mask length is ' + maskLength + ' whereas input is ' + bytes.length);
  }

  names = names || [];
  var offset = 0;
  return mask
    .map(function(decodeFn) {
      var current = bytes.slice(offset, offset += decodeFn.BYTES);
      return decodeFn(current);
    })
    .reduce(function(prev, cur, idx) {
      prev[names[idx] || idx] = cur;
      return prev;
    }, {});
};

if (typeof module === 'object' && typeof module.exports !== 'undefined') {
  module.exports = {
    unixtime: unixtime,
    uint8: uint8,
    uint16: uint16,
    temperature: temperature,
    humidity: humidity,
    latLng: latLng,
    bitmap: bitmap,
    rawfloat: rawfloat,
    decode: decode
  };
}