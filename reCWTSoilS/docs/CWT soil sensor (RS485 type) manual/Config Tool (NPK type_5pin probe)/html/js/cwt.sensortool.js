//�������ݴ�����
var DataRecvHandler = {
    //���յ���ַ����
    ReceivedAddrData: function (data, valueType) {
        valueType = Modbus.ValueType.Unsigned;
        this.slaveAddr = Modbus.GetIntValue(HexStrToBytes(data));
    },
    //���յ����õ�ַ�ķ���(���ϸ�)
    ReceivedSetAddrResponse: function (data, valueType) {
        //03 06 00 64 00 (03) 89 F6 
        this.slaveAddr = HexStrToBytes(data)[5];
    },
    //���յ��Ĵ�������
    ReceivedRegisterData: function (data, valueType) {
        this.regValue = Modbus.GetValue(HexStrToBytes(data),valueType);
    },
    //���յ�д���ݵĻ�Ӧ
    ReceivedWriteResponseData: function (data) {
        var code = HexStrToBytes(data)[1];
        alert("Write " + (code > 0x80 ? "fail" : "ok"));
    }
};