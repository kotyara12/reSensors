/*
var IEEE754 = {
    //16进制转single float
    //hexStr(eg:01 02 03 04)
    HexStrToFloat: function (hexStr) {
        hexStr = hexStr.replace(/\s+/g, "");
        if (hexStr == "") {
            return "";
        }
        if (hexStr == "00000000") {
            return 0;
        }
        if ((hexStr.length > 8) || (isNaN(parseInt(hexStr, 16)))) {
            return "Error";
        }
        if (hexStr.length < 8) {
            hexStr = hexStr.padStart(8, "0");
        }
        hexStr = parseInt(hexStr, 16).toString(2);
        hexStr = hexStr.padStart(32, "0");
        //float = (s==1?-1:1) * (1+M) * (2^(e-127))
        //注：M=(m0*2^(-1) + m1*2^(-2) + .. + mn*2^(-n))
        //0 100 0001 0 010 0000 0000 0000 0000 0000
        //s=>0=>+1
        //e=>100 0001 0=>130;
        //m=>010 0000 0000 0000 0000 0000=>0+1*2^(-2)+0...=>0.25
        //float = 1 * (1+0.25) * 8 = 10
        var s = hexStr.substring(0, 1);//IEEE754 符号位
        var e = hexStr.substring(1, 9);//IEEE754 指数
        var m = hexStr.substring(9);   //IEEE754 小数域(+1=>尾数)
        e = parseInt(e, 2) - 127;
        m = "1" + m;
        if (e >= 0) {
            m = m.substr(0, e + 1) + "." + m.substring(e + 1)
        }
        else {
            m = "0." + m.padStart(m.length - e - 1, "0");
        }
        if (m.indexOf(".") == -1) {
            m = m + ".0";
        }
        var a = m.split(".");
        var mi = parseInt(a[0], 2);
        var mf = 0;
        for (var i = 0; i < a[1].length; i++) {
            mf += parseFloat(a[1].charAt(i)) * Math.pow(2, -(i + 1));
        }
        m = parseInt(mi) + parseFloat(mf);
        if (s == 1) {
            m = 0 - m;
        }
        return m;
    },
    //single float to hex
    FloatToHexStr: function (fVal) {
        var s, e, m;
        s = fVal > 0 ? 0 : 1;
        fVal = Math.abs(fVal);
        m = fVal.toString(2);
        if (m >= 1) {
            if (m.indexOf(".") == -1) {
                m = m + ".0";
            }
            e = m.indexOf(".") - 1;
        }
        else {
            e = 1 - m.indexOf("1");
        }
        if (e >= 0) {
            m = m.replace(".", "");
        }
        else {
            m = m.substring(m.indexOf("1"));
        }
        if (m.length > 24) {
            m = m.substr(0, 24);
        }
        else {
            m = m.padEnd(24, "0");
        }
        m = m.substring(1);
        e = (e + 127).toString(2);
        e = e.padStart(8,"0");
        var r = parseInt(s + e + m, 2).toString(16);
        r = r.padStart(8, "0");
        return r;
    },

    //single float转byte[]
    FloatToBytes: function (fVal) {
        if (fVal == 0) return [0, 0, 0, 0];
        var s = fVal < 0 ? "1" : "0";//序号位
        var exponent = 127;
        if (decValue >= 2) {
            while (decValue >= 2) {
                exponent++;
                decValue /= 2;
            }
        }
        else if (decValue < 1) {
            while (decValue < 1) {
                exponent--;
                decValue *= 2;
                if (exponent == 0)
                    break;
            }
        }
        if (exponent != 0) decValue -= 1; else decValue /= 2;

        var fractionString = DecToBinTail(decValue, 23);
        var exponentString = DecToBinHead(exponent, 8);
        return Right('00000000' + parseInt(signString + exponentString + fractionString, 2).toString(16), 8);
    }
};
*/
/**
 * 写float值到byte[]
 * Writes an IEEE754 float to a byte array.
 * @param {!Array} buffer (bytes)
 * @param {number} value (float)
 * @param {number} offset (byte array)
 * @param {boolean} isLE (小端模式)
 * @param {number} mLen (小数域长度，32位单精度浮点数取=23)
 * @param {number} nBytes (字节长度，32位=4)
 * @inner
 */
function ieee754_write(buffer, value, offset, isLE, mLen, nBytes) {
    var e, m, c,
        eLen = nBytes * 8 - mLen - 1,
        eMax = (1 << eLen) - 1,
        eBias = eMax >> 1,
        rt = (mLen === 23 ? Math.pow(2, -24) - Math.pow(2, -77) : 0),
        i = isLE ? 0 : (nBytes - 1),
        d = isLE ? 1 : -1,
        s = value < 0 || (value === 0 && 1 / value < 0) ? 1 : 0;

    value = Math.abs(value);

    if (isNaN(value) || value === Infinity) {
        m = isNaN(value) ? 1 : 0;
        e = eMax;
    } else {
        e = Math.floor(Math.log(value) / Math.LN2);
        if (value * (c = Math.pow(2, -e)) < 1) {
            e--;
            c *= 2;
        }
        if (e + eBias >= 1) {
            value += rt / c;
        } else {
            value += rt * Math.pow(2, 1 - eBias);
        }
        if (value * c >= 2) {
            e++;
            c /= 2;
        }

        if (e + eBias >= eMax) {
            m = 0;
            e = eMax;
        } else if (e + eBias >= 1) {
            m = (value * c - 1) * Math.pow(2, mLen);
            e = e + eBias;
        } else {
            m = value * Math.pow(2, eBias - 1) * Math.pow(2, mLen);
            e = 0;
        }
    }

    for (; mLen >= 8; buffer[offset + i] = m & 0xff, i += d, m /= 256, mLen -= 8) { }

    e = (e << mLen) | m;
    eLen += mLen;
    for (; eLen > 0; buffer[offset + i] = e & 0xff, i += d, e /= 256, eLen -= 8) { }

    buffer[offset + i - d] |= s * 128;
}
/**
 * 从byte[]中获得float值
 * Reads an IEEE754 float from a byte array.
 * @param {!Array} buffer (bytes)
 * @param {number} offset (从byte array的哪一位开始)
 * @param {boolean} isLE (小端模式)
 * @param {number} mLen (小数域长度，32位单精度浮点数取=23)
 * @param {number} nBytes (字节长度，32位=4)
 * @returns {number} 浮点数
 * @inner
 */
function ieee754_read(buffer, offset, isLE, mLen, nBytes) {
    var e, m,
        eLen = nBytes * 8 - mLen - 1,
        eMax = (1 << eLen) - 1,
        eBias = eMax >> 1,
        nBits = -7,
        i = isLE ? (nBytes - 1) : 0,
        d = isLE ? -1 : 1,
        s = buffer[offset + i];

    i += d;

    e = s & ((1 << (-nBits)) - 1);
    s >>= (-nBits);
    nBits += eLen;
    for (; nBits > 0; e = e * 256 + buffer[offset + i], i += d, nBits -= 8) { }

    m = e & ((1 << (-nBits)) - 1);
    e >>= (-nBits);
    nBits += mLen;
    for (; nBits > 0; m = m * 256 + buffer[offset + i], i += d, nBits -= 8) { }

    if (e === 0) {
        e = 1 - eBias;
    } else if (e === eMax) {
        return m ? NaN : ((s ? -1 : 1) * Infinity);
    } else {
        m = m + Math.pow(2, mLen);
        e = e - eBias;
    }
    return (s ? -1 : 1) * m * Math.pow(2, e - mLen);
}