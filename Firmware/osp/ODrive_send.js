/********************************
 * 
 * Notes: ODrive 指令发送模板
 * 读取指令格式: "READ:参数1,READ:参数2,..."
 * 设置指令格式: "参数1:值1,参数2:值2,..."
 * 参数列表:
 *   READ: 读取参数
 *   ODRV_IBUS: 电流
 *   ODRV_M0_IBUS: 电机0电流
 *   ODRV_M1_IBUS: 电机1电流
 *   ODRV_TEMP: 温度
 *   ODRV_M0_TEMP: 电机0温度
 *   ODRV_M1_TEMP: 电机1温度
 *   ODRV_M0_SPEED: 电机0速度
 *   ODRV_M1_SPEED: 电机1速度
 *   ODRV_M0_HALL: 电机0霍尔状态
 *   ODRV_M1_HALL: 电机1霍尔状态
 *******************************/

(
    function main() {
        var codes = {
            READ:           0,
            ODRV_VBAT:      101,
            ODRV_IBUS:      102,
            ODRV_M0_IBUS:   103,
            ODRV_M1_IBUS:   104,
            ODRV_TEMP:      105,
            ODRV_M0_TEMP:   106,
            ODRV_M1_TEMP:   107,
            ODRV_M0_SPEED:  108,
            ODRV_M1_SPEED:  109,
            ODRV_M0_HALL:   110,
            ODRV_M1_HALL:   111,
            ODRV_M0_ANGLE:  112,
            ODRV_M1_ANGLE:  113,
            ODRV_AIBUS:     114,
            ODRV_M0_AIBUS:  115,
            ODRV_M1_AIBUS:  116,

            
            CUSTOM_0:       200,
            CUSTOM_1:       201,
            CUSTOM_2:       202,
            CUSTOM_3:       203,
            CUSTOM_4:       204,
            CUSTOM_5:       205,
            CUSTOM_6:       206,
            CUSTOM_7:       207,
            CUSTOM_8:       208,
            CUSTOM_9:       209,
        };
        
        var str = send.getString();
        var words = str.split(","); // 按逗号分隔
        for (var i = 0; i < words.length; i++) {
            var word = words[i].split(":"); // 按分号分隔
            var cmd = codes[word[0]];
            var value = 0;
            if (cmd == codes.READ) {
                value = codes[word[1]];
            } else {
                value = parseInt(word[1]);
            }
// FF FF 03 61 04 00 64 00 C8 CA 
            if (typeof cmd != undefined) {
                var buffer_out = new ArrayBuffer(10); // 创建一个长度为9字节的ArrayBuffer
                var buf_view = new Uint8Array(buffer_out);
                buf_view[0] = 0xFF; // 帧头
                buf_view[1] = 0xFF; // 帧头
                buf_view[2] = 0x03; // 剩余长度
                buf_view[3] = cmd; // 数据类型
                buf_view[4] = 04; // 数据长度
                buf_view[5] = (value >> 24); // data
                buf_view[6] = (value >> 16); // data
                buf_view[7] = (value >> 8); // data
                buf_view[8] = value; // data
                buf_view[9] = buf_view[2] ^ buf_view[3] ^ buf_view[4] ^ buf_view[5] ^ buf_view[6] ^ buf_view[7] ^ buf_view[8]; // 校验和
                console.log(buf_view);
                send.writeBytes(buf_view);
            }
        }
        return;
    }
)()
