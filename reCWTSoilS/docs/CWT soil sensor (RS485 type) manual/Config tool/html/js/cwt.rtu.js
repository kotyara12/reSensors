var Rtu = {
    GetRegisters: function (startAddr, step, count, name, regType, valueType, valueScale,write) {
        write = Defined(write) ? write : true;
        var regs = [];
        for (var i = 0; i < count; i++) {
            var item = {
                regAddr: startAddr + i * step,
                regName: (startAddr + i * step) + ": " + name + i,
                regType: regType,
                valueType: valueType,
                valueScale: valueScale,
                write: write
            }
            regs.push(item);
        }
        return regs;
    }
}