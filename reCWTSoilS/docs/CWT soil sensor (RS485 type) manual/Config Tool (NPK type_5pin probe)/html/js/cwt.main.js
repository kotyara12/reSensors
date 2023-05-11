var sensor;
/**
 * 更新端口(系统调用)
 * 系统传参：string[]=>{"COM1","COM2",...}
 * */
function UpdateComPort() {
    var comPort = Id("comPort");
    comPort.options.length = 0;
    for (var i in arguments) {
        comPort.options.add(new Option(arguments[i], arguments[i]));
    }
    //comPort.selectedIndex = comPort.options.length - 1;
}
/**
 * 修改端口状态(系统调用)
 * @param {any} state 
 */
function ChangePortState(state) {
    var btnOpen = Id("btnOpen");
    btnOpen.innerHTML = state ? "Close" : "Open";
    btnOpen.style.color = state ? "green" : "red";
}
/**
 * 接收到数据(系统调用)
 * 系统传参:hex string=>xx xx xx xx ...
 * */
function ReceivedData() {
    AppendLog("RX:" + arguments[0]);
    if (Defined(sensor.ReceivedData) && sensor.ReceivedData != null) {
        sensor.ReceivedData(arguments[0], IdIntValue("valueType"));
        Id("slaveAddr").value = sensor.slaveAddr;
        var scale = IdValue("valueScale");
        if (Defined(sensor.regValue) && sensor.regValue !== null) {
            if (scale.indexOf(".") > 0) {//float
                Id("regValue").value = (sensor.regValue * parseFloat(scale)).toFixed(2);
            } else {//int
                Id("regValue").value = sensor.regValue * parseInt(scale);
            }
            sensor.regValue = null;
        }
    }
}

/**
 * 添加数据到日志显示
 * @param {any} msg 数据
 */
function AppendLog(msg) {
    textLog = Id("textLog");
    textLog.appendChild(document.createTextNode(msg + "\n"));
    textLog.scrollTop = textLog.scrollHeight;
}

/**
 * 加载传感器
 * */
function InitSensors() {
    var sensorType = Id("sensorType");
    sensorType.options.length = 0;
    for (var i in Sensors) {
        sensorType.options.add(new Option(Sensors[i].name, i));
    }
    sensorType.onchange();
}

/**
 * 改变寄存器下拉值时修改类型和值类型
 * */
function ChangeRegTypes() {
    var sensor = Sensors[IdValue("sensorType")];
    var regIndex = Id("regAddr").selectedIndex;
    Id("regType").value = sensor.Registers[regIndex].regType;
    Id("valueType").value = sensor.Registers[regIndex].valueType;
    Id("regType").onchange();
    //转换比例
    Id("valueScale").value = sensor.Registers[regIndex].valueScale;
    //是否可设置
    DisplayById("btnWriteRegister", sensor.Registers[regIndex].write);
}

/**
 * 加载完成
 * 1.修改程序名称
 * 2.端口更新
 * */
function LoadCompleted() {
    JsInvoke("LoadCompleted", document.title);
}
//页面加载完成
window.onload = function () {

    LoadCompleted();
    //刷新事件
    Id("refresh").addEventListener("click", function () {
        JsInvoke("Refresh");
    });
    //打开端口
    Id("btnOpen").onclick = function () {
        JsInvoke("OpenPort", IdsValue("comPort", "bps", "parity"));
    }
    //传感器类型选择
    Id("sensorType").onchange = function () {
        sensor = Sensors[this.value];
        Id("sensorImg").src = "images/sensor/" + sensor.img;
        var tips = Defined(sensor.Tips) ? sensor.Tips : "";
        Id("tips").innerHTML = tips;
        Id("sensorImg").title = tips;
        Id("slaveAddr").value = sensor.slaveAddr;
        //寄存器地址，先删除然后创建(因为类型变了)
        var regAddr = Id("regAddr");
        var parentNode = regAddr.parentNode;
        parentNode.removeChild(regAddr);
        if (Defined(sensor.Init)) sensor.Init();
        if (sensor.Registers instanceof Array) {
            regAddr = document.createElement("select");
            for (var i = 0; i < sensor.Registers.length; i++) {
                regAddr.options.add(new Option(sensor.Registers[i].regName, sensor.Registers[i].regAddr));
            }
            regAddr.id = "regAddr";
            parentNode.appendChild(regAddr);
            regAddr.onchange = function () {
                ChangeRegTypes();
            };
            DisableById("regType", true);
            DisableById("valueType", true);
            DisableById("valueScale", true);
            regAddr.onchange();
        } else {
            regAddr = document.createElement("input");
            regAddr.placeholder = "base 1";
            Id("valueScale").value = "1";
            DisableById("regType", false);
            DisableById("valueType", false);
            DisableById("valueScale", false);
            regAddr.id = "regAddr";
            parentNode.appendChild(regAddr);
            Id("regType").onchange();
        }
        DisplayById("btnReadAddr", sensor.QueryAddrHex);
        DisplayById("btnWriteAddr", sensor.displayBtnWriteAddr);
    }
    //地址设置
    Id("slaveAddr").onblur = function () {
        sensor.slaveAddr = parseInt(this.value);
    }
    //寄存器类型选择
    Id("regType").onchange = function () {
        var enableSet = [true, false, false, true];
        if ((Id("regAddr").tagName != "SELECT")) {
            DisplayById("btnWriteRegister", enableSet[this.selectedIndex]);
        }
        //COIL/DI将值类型改为unsigned
        var regType = IdValue("regType");
        if (regType < Modbus.RegType.INPUT) {
            Id("valueType").value = Modbus.ValueType.Unsigned;
        }
        if (this.getAttribute("disabled") != "disabled") {
            //只有
            DisableById("valueType", regType < Modbus.RegType.INPUT);
        }
    }
    //读取传感器地址
    Id("btnReadAddr").onclick = function () {
        sensor.ReceivedData = DataRecvHandler.ReceivedAddrData;
        var msg = sensor.QueryAddrHex;
        if (typeof (msg) != "undefined") {
        		AppendLog("TX:"+msg);
            SendHex(msg);
        }
    }
    //写地址
    Id("btnWriteAddr").onclick = function () {
        var newAddr = parseInt(IdValue("slaveNewAddr"));
        sensor.slaveAddr = parseInt(IdValue("slaveAddr"));
        if (!isNaN(sensor.slaveAddr)) {
            if (newAddr != NaN) {
                var data = Modbus.GetWriteSlaveAddrData(sensor.slaveAddr, newAddr, sensor.slaveRegAddr);
                data = BytesToHexStr(data);
                SendHex(data);
                AppendLog("TX:"+data);
                sensor.ReceivedData = DataRecvHandler.ReceivedSetAddrResponse;
            } else {
                alert("Address error!");
            }
        } else {
            if (!CheckNotNull("slaveAddr", "regAddr")) return;
            //alert("Please read first.");
        }
    }
    //读寄存器
    Id("btnReadRegister").onclick = function () {
        if (!CheckNotNull("slaveAddr", "regAddr")) return;
        sensor.ReceivedData = DataRecvHandler.ReceivedRegisterData;
        var addr = parseInt(IdValue("slaveAddr"));
        var regAddr = parseInt(IdValue("regAddr"));
        var regType = parseInt(IdValue("regType"));
        var valueType = parseInt(IdValue("valueType"));
        var data = Modbus.GetReadRegData(addr, regAddr, regType, valueType);
        data = BytesToHexStr(data);
        SendHex(data);
        AppendLog("TX:" + data);
    }
    //写寄存器值
    Id("btnWriteRegister").onclick = function () {
        if (!CheckNotNull("slaveAddr", "regAddr", "regValue")) return;
        sensor.ReceivedData = DataRecvHandler.ReceivedWriteResponseData;
        sensor.regValue = null;
        var addr = parseInt(IdValue("slaveAddr"));
        var regAddr = parseInt(IdValue("regAddr"));
        var regType = parseInt(IdValue("regType"));
        var value = IdValue("regValue");
        var valueType = parseInt(IdValue("valueType"));
        var data = Modbus.GetWriteRegData(addr, regAddr, regType, value, valueType);
        data = BytesToHexStr(data);
        SendHex(data);
        AppendLog("TX:" + data);
    }
    //清空日志
    Id("btnClearLog").onclick = function () {
        Id("textLog").value = '';
    }
    //发送hex
    // Id("sendHex").onclick = function () {
    //     if (!CheckNotNull("hexData")) return;
    //     var data = IdValue("hexData");
    //     SendHex(data);
    //     AppendLog("TX:"+data);
    // }
    //值类型变更
    Id("valueType").onchange = function () {
        Id("regValue").value = "";
    }
    InitSensors();
    Id("regType").onchange();
}
/**
 * 异常处理
 * @param {any} errorMessage
 * @param {any} scriptURI
 * @param {any} lineNumber
 * @param {any} columnNumber
 * @param {any} errorObj
 */
window.onerror = function (errorMessage, scriptURI, lineNumber, columnNumber, errorObj) {
    alert(errorMessage);
    return true;
}
