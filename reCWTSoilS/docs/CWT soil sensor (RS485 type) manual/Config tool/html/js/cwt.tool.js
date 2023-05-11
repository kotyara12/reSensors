//1.调用系统方法,下面是可用方法:
//打开端口:         OpenPort,[COM1,9600,None]
//发送字符串:       SendData,data
//发送16进制字符串: SendHex,hexData
//刷新html页面:     Refresh[,null]
//通知页面加载完成: LoadCompleted,titleName
//2.附:系统会调用的js方法:
//更新端口:         UpdateComPort(string[]=>"COM1","COM2",...)
//端口状态变更:     ChangePortState(bool)
//接收到数据(Hex):  ReceivedData(hexString=>"xx xx xx xx ...")
function JsInvoke(fun, msg) {
    if (typeof (window.external.JsInvoke) != "undefined") {
        window.external.JsInvoke(fun, msg);
    } else {
        alert(fun + ":" + msg);
    }
}

//替换掉空格
function ReplaceSpace(data) {
    return data.replace(/\s/g, "");
}

//交换高低位
function ReverseHighLow(hexStr) {
    hexStr = hexStr.replace(/\s/g, "");
    len = hexStr.length;
    if (len % 2 == 0) {
        hexStr = hexStr.substring(len / 2) + hexStr.substring(0, len / 2);
    }
    return hexStr;
}

//发送16进制数据
function SendHex(msg) {
    JsInvoke("SendHex", msg);
}

//字节数组转十六进制字符串
function BytesToHexStr(arr) {
    var str = "";
    for (var i = 0; i < arr.length; i++) {
        var tmp = arr[i];
        tmp = tmp < 0 ? tmp + 256 : tmp;//小于0需要处理
        tmp = tmp.toString(16);
        tmp = (tmp.length == 1 ? "0" : "") + tmp;
        str += tmp;
        str += " ";
    }
    return str;
}

//16进制字符串转整数数组
function HexStrToBytes(hex) {
    hex = hex.replace(/\s/g, "")
    var len = hex.length;
    var value = [];
    for (var i = 0; i < len; i += 2) {
        value[i / 2] = parseInt(hex[i], 16) * 16 + parseInt(hex[i + 1], 16);
    }
    return value;
}

//整数转16进制字符串
function IntToHexStr(num) {
    return num.toString(16);
}
//16进制字符串转int
function HexStrToInt(hex) {
    hex = hex.replace(/\s/g, "");
    return parseInt(hex, 16);
}

//from ES6
if (!String.prototype.repeat) {
    String.prototype.repeat = function (count) {
        'use strict';
        if (this == null)
            throw new TypeError('can\'t convert ' + this + ' to object');

        var str = '' + this;
        // To convert string to integer.
        count = +count;
        // Check NaN
        if (count != count)
            count = 0;

        if (count < 0)
            throw new RangeError('repeat count must be non-negative');

        if (count == Infinity)
            throw new RangeError('repeat count must be less than infinity');

        count = Math.floor(count);
        if (str.length == 0 || count == 0)
            return '';

        // Ensuring count is a 31-bit integer allows us to heavily optimize the
        // main part. But anyway, most current (August 2014) browsers can't handle
        // strings 1 << 28 chars or longer, so:
        if (str.length * count >= 1 << 28)
            throw new RangeError('repeat count must not overflow maximum string size');

        var maxCount = str.length * count;
        count = Math.floor(Math.log(count) / Math.log(2));
        while (count) {
            str += str;
            count--;
        }
        str += str.substring(0, maxCount - str.length);
        return str;
    }
}

//from ES6
// https://github.com/uxitten/polyfill/blob/master/string.polyfill.js
// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/String/padStart
//字符串前补充
if (!String.prototype.padStart) {
    String.prototype.padStart = function padStart(targetLength, padString) {
        targetLength = targetLength >> 0; //truncate if number, or convert non-number to 0;
        padString = String(padString !== undefined ? padString : ' ');
        if (this.length >= targetLength) {
            return String(this);
        } else {
            targetLength = targetLength - this.length;
            if (targetLength > padString.length) {
                padString += padString.repeat(targetLength / padString.length); //append to original to ensure we are longer than needed
            }
            return padString.slice(0, targetLength) + String(this);
        }
    };
}

// https://github.com/uxitten/polyfill/blob/master/string.polyfill.js
// https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/String/padEnd
//字符串后补充
if (!String.prototype.padEnd) {
    String.prototype.padEnd = function padEnd(targetLength, padString) {
        targetLength = targetLength >> 0; // truncate if number or convert non-number to 0;
        padString = String(padString !== undefined ? padString : ' ');
        if (this.length >= targetLength) {
            return String(this);
        }
        else {
            targetLength = targetLength - this.length;
            if (targetLength > padString.length) {
                padString += padString.repeat(targetLength / padString.length); // append to original to ensure we are longer than needed
            }
            return String(this) + padString.slice(0, targetLength);
        }
    };
}


//无符号数转符号数
//value:要转换的值
//bit:位数,16/32
function UnsignedToSigned(value, bit) {
    if (value > (1 << (bit - 1) - 1)) {//负数
        //去掉符号位，位取反，+1，加上符号位
        var result = value.toString(2).padStart(bit, "0");
        result = result.substring(result.length - (bit - 1));
        var tmp = "";
        for (var i = 0; i < result.length; i++) {
            tmp += result[i] == "1" ? "0" : "1";
        }
        result = parseInt(tmp, 2);
        result = result + 1;
        result = result % (1 << (bit - 1));
        result *= -1;
        return result;
    }
    return value;
}

/**
 * 有符号数转byte数组
 * @param {any} num 要转换的数
 * @param {any} bit 位数,必须16的倍数
 */
function SignedToBytes(num, bit) {
    //bit为32如果使用1<<32会为1，所以使用下面的方法
    var bitMaxValue = 1;
    for (var i = 0; i < bit / 16; i++) {
        bitMaxValue *= (1 << 16);
    }
    num %= bitMaxValue;
    var bNum = num >= 0 ? num : (bitMaxValue + num);
    var bytes = [];
    var len = bit / 8;
    for (var i = 0; i < len; i++) {
        bytes[i] = parseInt((bNum / Math.pow(256, len - i - 1)) % 256);
    }
    return bytes;
}

//根据ID返回对象
function Id(id) {
    return document.getElementById(id);
}
//根据ID获得值
function IdValue(id) {
    return document.getElementById(id).value;
}
//根据ID获得int值
function IdIntValue(id) {
    return parseInt(IdValue(id));
}
//返回多个id的值数组
function IdsValue() {
    var values = [];
    for (var i = 0; i < arguments.length; i++) {
        values[i] = IdValue(arguments[i]);
    }
    return values;
}
//根据id数组检测value不为空，全不为空返回true
function CheckNotNull() {
    for (var index in arguments) {
        if (IdValue(arguments[index]).length == 0) {
            Id(arguments[index]).focus();
            return false;
        }
    }
    return true;
}
//变量是定义
function Defined(obj) {
    return typeof (obj) != "undefined";
}

//是否是中文
function IsChinese() {
    var language = null;
    if (navigator.appName == 'Netscape') {
        language = navigator.language;
    }
    else {
        language = navigator.browserLanguage;
    }
    return language.indexOf('zh') == 0;
}

/**
 * 根据id控制view的显示
 * @param {any} id
 * @param {any} display (true/false)
 */
function DisplayById(id, display) {
    Id(id).style.display = display ? "inline" : "none";
}

/**
 * 根据id控制view是否用可
 * @param {any} id
 * @param {any} disable (true/false)
 */
function DisableById(id, disable) {
    if (disable) {
        Id(id).setAttribute("disabled", "disabled");
    } else {
        Id(id).removeAttribute("disabled");
    }
}