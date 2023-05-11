//接收数据处理方法
var DataRecvHandler = {
    //接收到地址数据
    ReceivedAddrData: function (data, valueType) {
        valueType = Modbus.ValueType.Unsigned;
        this.slaveAddr = Modbus.GetIntValue(HexStrToBytes(data));
    },
    //接收到设置地址的反馈(不严格)
    ReceivedSetAddrResponse: function (data, valueType) {
        //03 06 00 64 00 (03) 89 F6 
        this.slaveAddr = HexStrToBytes(data)[5];
    },
    //接收到寄存器数据
    ReceivedRegisterData: function (data, valueType) {
        this.regValue = Modbus.GetValue(HexStrToBytes(data),valueType);
    },
    //接收到写数据的回应
    ReceivedWriteResponseData: function (data) {
        var code = HexStrToBytes(data)[1];
        alert("Write " + (code > 0x80 ? "fail" : "ok"));
    }
};