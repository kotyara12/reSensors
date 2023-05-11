var Modbus = {
    Base: 1,//寄存器地址从1开始
    RegType: {//寄存器类型
        COIL: 0,
        DI: 1,
        INPUT: 3,
        KEEP: 4
    },
    ValueType: {//值类型
        Signed: 0,
        Unsigned: 1,
        Long: 2,
        LongReverse: 3,
        Float: 4,
        FloatReverse: 5,
    },
    //值类型寄存器长度(对应上面的ValueType)
    ValueTypeRegLen: [1, 1, 2, 2, 2, 2],
    //读寄存器功能码(序号与上面的RegType值对应)
    ReadFunCodes: [0x01, 0x02, 0, 0x04, 0x03],
    //写单个寄存器功能码(序号与上面的RegType值对应)
    WriteSingleFunCodes: [0x05, 0, 0, 0, 0x06],
    //写多个寄存器功能码(序号与上面的RegType值对应)
    WriteMultiFunCodes: [0x0F, 0, 0, 0, 0x10],
    /**
     * 获得CRC检验
     * @param {any} bytes byte[]
     * @param {any} len byte[]计算CRC的部分的长度
     */
    GetCRC16: function (bytes, len) {
        // 预置 1 个 16 位的寄存器为十六进制FFFF, 称此寄存器为 CRC寄存器。
        var crc = 0xFFFF;
        var i, j;
        for (i = 0; i < len; i++) {
            // 把第一个 8 位二进制数据 与 16 位的 CRC寄存器的低 8 位相异或, 把结果放于 CRC寄存器
            crc = ((crc & 0xFF00) | (crc & 0x00FF) ^ (bytes[i] & 0xFF));
            for (j = 0; j < 8; j++) {
                // 把 CRC 寄存器的内容右移一位( 朝低位)用 0 填补最高位, 并检查右移后的移出位
                if ((crc & 0x0001) > 0) {
                    // 如果移出位为 1, CRC寄存器与多项式A001进行异或
                    crc = crc >> 1;
                    crc = crc ^ 0xA001;
                }
                else
                    // 如果移出位为 0,再次右移一位
                    crc = crc >> 1;
            }
        }
        //return crc;//整数
        return [crc % 0x100, parseInt(crc / 0x100)];
    },
    /**
     * byte[]交换前后字节(xx XX 或 xx xx XX XX)
     * @param {any} bytes
     */
    ReverseByte: function (bytes) {
        var len = bytes.length / 2;
        bytes = bytes.slice(len).concat(bytes.slice(0, len));
        return bytes;
    },

    /**
     * 获得值部分的byte[]
     * @param {any} bytes
     */
    GetValueBytes: function (bytes) {
        var len = bytes[2];
        return bytes.slice(3, 3 + len);
    },
    //获得值的16进制字符串
    //bytes:[]
    GetValueHexStr: function (bytes) {
        return BytesToHexStr(this.GetValueBytes(bytes));
    },

    /**
     * 获得值
     * @param {any} bytes (byte[])
     * @param {any} valueType (值类型)
     */
    GetValue: function (bytes, valueType) {
        var value;
        switch (valueType) {
            case this.ValueType.Float:
            case this.ValueType.FloatReverse:
                value = this.GetFloatValue(bytes, valueType == this.ValueType.FloatReverse);
                break;
            case this.ValueType.Signed:
                value = this.GetIntValue(bytes);
                value = UnsignedToSigned(value, 16);
                break;
            case this.ValueType.Long:
            case this.ValueType.LongReverse:
                value = this.GetLongValue(bytes, valueType == this.ValueType.LongReverse);
                break;
            default:
                value = this.GetIntValue(bytes);
        }
        return value;
    },
    /**
     * 获得int值(无符号)
     * @param {any} bytes
     */
    GetIntValue: function (bytes) {
        var value = 0;
        var len = bytes[2];
        var data = this.GetValueBytes(bytes);
        for (var i = 0; i < len; i++) {
            value += data[i] * Math.pow(0x100, len - 1 - i);
        }
        return value;
    },
    /**
     * 获得float值
     * @param {any} data (byte[])
     * @param {any} reverse (FloatReverse值?)
     */
    GetFloatValue: function (bytes, floatReverse) {
        value = this.GetValueBytes(bytes);
        //Float需要交换前后2位xx xx XX XX
        if (!floatReverse) value = this.ReverseByte(value);
        return ieee754_read(value, 0, false, 23, 4).toFixed(2);
    },
    /**
     * 获得Long值
     * @param {any} bytes (byte[])
     * @param {any} longReverse (LongReverse值?)
     */
    GetLongValue: function (bytes, longReverse) {
        value = Modbus.GetValueBytes(bytes);
        if (!longReverse) value = this.ReverseByte(value);
        value = BytesToHexStr(value);
        value = HexStrToInt(value);
        //return UnsignedToSigned(value, 32);
        return value;
    },

    /**
     * 写(从机/设备)地址
     * @param {any} oldAddr 旧地址
     * @param {any} newAddr 新地址
     * @param {any} regAddr 寄存器位置
     */
    GetWriteSlaveAddrData: function (oldAddr, newAddr, regAddr) {
        var data = [];
        data[0] = oldAddr;//从机地址(原来的)
        data[1] = 0x06;//功能码
        data[2] = parseInt(regAddr / 0x100);//寄存器高位
        data[3] = parseInt(regAddr % 0x100);//寄存器低位
        data[4] = 0x00;//数值高位；地址只会是0-255，高位用0
        data[5] = newAddr % 256;//数值低位
        data = data.concat(this.GetCRC16(data, data.length));
        return data;
    },
    /**
     * 获得“读寄存器”的数据
     * @param {any} addr 从机地址
     * @param {any} regAddr 开始寄存器地址
     * @param {any} regType 寄存器类型(COIL/KEEP)
     * @param {any} valueType 值类型
     */
    GetReadRegData: function (addr, regAddr, regType, valueType) {
        var data = [];
        data[0] = addr;//从机地址
        data[1] = this.ReadFunCodes[regType];//功能码
        regAddr -= this.Base;
        data[2] = parseInt(regAddr / 0x100);//寄存器高位
        data[3] = parseInt(regAddr % 0x100);//寄存器低位
        var count = this.ValueTypeRegLen[valueType];
        //COIL/DI需要1个寄存器长度
        count = regType < this.RegType.INPUT ? 1 : count;
        data[4] = parseInt(count / 0x100);//长度高位
        data[5] = count % 256;//长度低位
        data = data.concat(this.GetCRC16(data, data.length));
        return data;
    },
    /**
     * 获得“写寄存器”的数据(最多2个寄存器值(32位值))
     * @param {any} addr 从机地址
     * @param {any} regAddr 开始寄存器地址
     * @param {any} regType 寄存器类型(COIL/KEEP)
     * @param {any} value 要写的值
     * @param {any} valueType 值类型
     */
    GetWriteRegData: function (addr, regAddr, regType, value, valueType) {
        var data = [];
        regAddr -= this.Base;
        data[0] = addr;//从机地址
        data[2] = parseInt(regAddr / 0x100);//寄存器高位
        data[3] = parseInt(regAddr % 0x100);//寄存器低位
        //功能码
        var count = this.ValueTypeRegLen[valueType];
        count = regType == this.RegType.COIL ? 1 : count;
        if (count == 1) {
            //写单个:[地址*1] [功能码*1] [寄存器地址*2] [值*2] [CRC*2]
            data[1] = this.WriteSingleFunCodes[regType];
            //COIL ON
            if (regType == this.RegType.COIL && value != 0) {
                value = 0xFF00;
            }
        } else {
            //写多个:[地址*1] [功能码*1] [寄存器地址*2] [寄存器数量*2] [值长度(n)*1] [值*n] [CRC*2] 
            data[1] = this.WriteMultiFunCodes[regType];
            data = data.concat(this.ChangeValueToBytes(count, this.ValueType.Signed));//寄存器数量
            data.push(count * 2);//值的字节长度
        }
        //值
        data = data.concat(this.ChangeValueToBytes(value, valueType));
        //+CRC
        data = data.concat(this.GetCRC16(data, data.length));
        return data;
    },
    /**
     * 将值转为字节数组
     * @param {any} value 值
     * @param {any} valueType 值类型
     */
    ChangeValueToBytes: function (value, valueType) {
        var bytes = [];
        switch (valueType) {
            case this.ValueType.Signed:
            case this.ValueType.Unsigned:
                value = parseInt(value);
                bytes = SignedToBytes(value, 16);
                break;
            case this.ValueType.Long:
                value = parseInt(value);
                bytes = SignedToBytes(value, 32);
                bytes = this.ReverseByte(bytes);
                break;
            case this.ValueType.LongReverse:
                value = parseInt(value);
                bytes = SignedToBytes(value, 32);
                break;
            case this.ValueType.Float:
                bytes = [];
                value = parseFloat(value);
                ieee754_write(bytes, value, 0, false, 23, 4);
                bytes = this.ReverseByte(bytes);
                break;
            case this.ValueType.FloatReverse:
                bytes = [];
                value = parseFloat(value);
                ieee754_write(bytes, value, 0, false, 23, 4);
                break;
            default:
                alert("Value type error!");
                break;
        }
        return bytes;
    }
};
