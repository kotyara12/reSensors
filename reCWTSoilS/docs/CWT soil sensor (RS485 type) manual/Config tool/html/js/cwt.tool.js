//1.����ϵͳ����,�����ǿ��÷���:
//�򿪶˿�:         OpenPort,[COM1,9600,None]
//�����ַ���:       SendData,data
//����16�����ַ���: SendHex,hexData
//ˢ��htmlҳ��:     Refresh[,null]
//֪ͨҳ��������: LoadCompleted,titleName
//2.��:ϵͳ����õ�js����:
//���¶˿�:         UpdateComPort(string[]=>"COM1","COM2",...)
//�˿�״̬���:     ChangePortState(bool)
//���յ�����(Hex):  ReceivedData(hexString=>"xx xx xx xx ...")
function JsInvoke(fun, msg) {
    if (typeof (window.external.JsInvoke) != "undefined") {
        window.external.JsInvoke(fun, msg);
    } else {
        alert(fun + ":" + msg);
    }
}

//�滻���ո�
function ReplaceSpace(data) {
    return data.replace(/\s/g, "");
}

//�����ߵ�λ
function ReverseHighLow(hexStr) {
    hexStr = hexStr.replace(/\s/g, "");
    len = hexStr.length;
    if (len % 2 == 0) {
        hexStr = hexStr.substring(len / 2) + hexStr.substring(0, len / 2);
    }
    return hexStr;
}

//����16��������
function SendHex(msg) {
    JsInvoke("SendHex", msg);
}

//�ֽ�����תʮ�������ַ���
function BytesToHexStr(arr) {
    var str = "";
    for (var i = 0; i < arr.length; i++) {
        var tmp = arr[i];
        tmp = tmp < 0 ? tmp + 256 : tmp;//С��0��Ҫ����
        tmp = tmp.toString(16);
        tmp = (tmp.length == 1 ? "0" : "") + tmp;
        str += tmp;
        str += " ";
    }
    return str;
}

//16�����ַ���ת��������
function HexStrToBytes(hex) {
    hex = hex.replace(/\s/g, "")
    var len = hex.length;
    var value = [];
    for (var i = 0; i < len; i += 2) {
        value[i / 2] = parseInt(hex[i], 16) * 16 + parseInt(hex[i + 1], 16);
    }
    return value;
}

//����ת16�����ַ���
function IntToHexStr(num) {
    return num.toString(16);
}
//16�����ַ���תint
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
//�ַ���ǰ����
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
//�ַ����󲹳�
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


//�޷�����ת������
//value:Ҫת����ֵ
//bit:λ��,16/32
function UnsignedToSigned(value, bit) {
    if (value > (1 << (bit - 1) - 1)) {//����
        //ȥ������λ��λȡ����+1�����Ϸ���λ
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
 * �з�����תbyte����
 * @param {any} num Ҫת������
 * @param {any} bit λ��,����16�ı���
 */
function SignedToBytes(num, bit) {
    //bitΪ32���ʹ��1<<32��Ϊ1������ʹ������ķ���
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

//����ID���ض���
function Id(id) {
    return document.getElementById(id);
}
//����ID���ֵ
function IdValue(id) {
    return document.getElementById(id).value;
}
//����ID���intֵ
function IdIntValue(id) {
    return parseInt(IdValue(id));
}
//���ض��id��ֵ����
function IdsValue() {
    var values = [];
    for (var i = 0; i < arguments.length; i++) {
        values[i] = IdValue(arguments[i]);
    }
    return values;
}
//����id������value��Ϊ�գ�ȫ��Ϊ�շ���true
function CheckNotNull() {
    for (var index in arguments) {
        if (IdValue(arguments[index]).length == 0) {
            Id(arguments[index]).focus();
            return false;
        }
    }
    return true;
}
//�����Ƕ���
function Defined(obj) {
    return typeof (obj) != "undefined";
}

//�Ƿ�������
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
 * ����id����view����ʾ
 * @param {any} id
 * @param {any} display (true/false)
 */
function DisplayById(id, display) {
    Id(id).style.display = display ? "inline" : "none";
}

/**
 * ����id����view�Ƿ��ÿ�
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