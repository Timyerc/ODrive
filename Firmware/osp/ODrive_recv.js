/********************************
 * 
 * Notes: 
 * 
 * 
 * 
 * 
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
      
      code_to_name = {};

      for (var key in codes) {
        var value = codes[key];
        code_to_name[value] = key + "";
      }
      
      var message_decode = function (data) {
        var data = new Uint8Array(data);
        
        if (typeof message_state == 'undefined') {
            message_state = 0;
        }
        
        if (typeof message_length_received == 'undefined') {
            message_length_received = 0;
        }
      
        for (var i = 0; i < data.length; i++) {
          switch (message_state) {
            case 0: // sync char 1
              if (data[i] == 0xFF) { // $
                message_state++;
              }
              break;
            case 1: // sync char 2
              if (data[i] == 0xFF) { // M
                message_state++;
              } else { // restart and try again
                message_state = 0;
              }
              break;
            case 2:
              message_code = data[i]; // code
              message_checksum = data[i];
              message_state++;
              break;
            case 3:
              message_length_expected = data[i]; // data length
              message_checksum ^= data[i];
              message_buffer = new ArrayBuffer(message_length_expected);
              message_buffer_uint8_view = new Uint8Array(message_buffer);
              
              if (message_length_expected != 0) { // standard message
                message_state++;
              } else { // MSP_ACC_CALIBRATION, etc...
                message_state += 2;
              }
              break;
            case 4: // data / payload
              message_buffer_uint8_view[message_length_received] = data[i];
              message_checksum ^= data[i];
              message_length_received++;
              
              if (message_length_received >= message_length_expected) {
                message_state++;
              }
              break;
            case 5: // CRC
              if (message_checksum == data[i]) {
                // process data
                var name = code_to_name[message_code];
                console.log(name);
                message_decode_payload(message_code, message_buffer);
              }
              // Reset variables
              message_length_received = 0;
              message_state = 0;           
              break;
          }
        }
      };
      
      var message_decode_payload = function (msp_code, data) {
        var view = new DataView(data, 0);
      
        var msp_code_name = code_to_name[msp_code];
        var payload = {};
        var out;
      
      
        var data_array_unit_parse = function (data, max, size) {
          size = size || 2;
      
          var size_to_getter = {
            1: 'getUint8',
            2: 'getUint16',
            4: 'getUint32'
          };
          var getter = size_to_getter[size];
          max = max || -1;
      
          var res = [];
          var view = new DataView(data, 0);
      
          for (var i = 0; i < data.byteLength; i += size) {
            var index = i / size;
            res[index] = view[getter](i, 1);
      
            if (max == index)
              return res;
          }
      
          return res;
        };
        
      
        switch (msp_code) {
          case codes.ODRV_VBAT:
            payload.vbat = view.getInt32(0, 1);
            out = 'vbat=' + (payload.vbat).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_M0_SPEED:
            payload.m0_speed = view.getInt32(0, 1);
            out = 'm0_speed=' + (payload.m0_speed).toString()  + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_M1_SPEED:
            payload.m1_speed = view.g
            payload.m1_speed = view.getInt32(0, 1);
            out = 'm1_speed=' + (payload.m1_speed).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_IBUS:
            payload.ibus = view.getInt32(0, 1);
            out = 'ibus=' + (payload.ibus).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_M0_IBUS:
            payload.m0_ibus = view.getInt32(0, 1);
            out = 'm0_ibus=' + (payload.m0_ibus).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_M1_IBUS:
            payload.m1_ibus = view.getInt32(0, 1);
            out = 'm1_ibus=' + (payload.m1_ibus).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_M0_HALL:
            payload.m0_hall = view.getInt32(0, 1);
            out = 'm0_hall=' + (payload.m0_hall).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.ODRV_M1_HALL:
            payload.m1_hall = view.getInt32(0, 1);
            out = 'm1_hall=' + (payload.m1_hall).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_TEMP:
            payload.temp = view.getInt32(0, 1);
            out = 'temp=' + (payload.temp).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_M0_TEMP:
            payload.m0_temp = view.getInt32(0, 1);
            out = 'm0_temp=' + (payload.m0_temp).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_M1_TEMP:
            payload.m1_temp = view.getInt32(0, 1);
            out = 'm1_temp=' + (payload.m1_temp).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_M0_ANGLE:
            payload.m0_angle = view.getInt32(0, 1);
            out = 'm0_angle=' + (payload.m0_angle).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_M1_ANGLE:
            payload.m1_angle = view.getInt32(0, 1);
            out = 'm1_angle=' + (payload.m1_angle).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_AIBUS:
            payload.aibus = view.getInt32(0, 1);
            out = 'aibus=' + (payload.aibus).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_M0_AIBUS:
            payload.m0_aibus = view.getInt32(0, 1);
            out = 'm0_aibus=' + (payload.m0_aibus).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            case codes.ODRV_M1_AIBUS:
            payload.m1_aibus = view.getInt32(0, 1);
            out = 'm1_aibus=' + (payload.m1_aibus).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
            




          case codes.CUSTOM_0:
            payload.custom_0 = view.getInt32(0, 1);
            out = 'custom_0=' + (payload.custom_0).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_1:
            payload.custom_1 = view.getInt32(0, 1);
            out = 'custom_1=' + (payload.custom_1).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_2:
            payload.custom_2 = view.getInt32(0, 1);
            out = 'custom_2=' + (payload.custom_2).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_3:
            payload.custom_3 = view.getInt32(0, 1);
            out = 'custom_3=' + (payload.custom_3).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_4:
            payload.custom_4 = view.getInt32(0, 1);
            out = 'custom_4=' + (payload.custom_4).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_5:
            payload.custom_5 = view.getInt32(0, 1);
            out = 'custom_5=' + (payload.custom_5).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_6:
            payload.custom_6 = view.getInt32(0, 1);
            out = 'custom_6=' + (payload.custom_6).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_7:
            payload.custom_7 = view.getInt32(0, 1);
            out = 'custom_7=' + (payload.custom_7).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_8:
            payload.custom_8 = view.getInt32(0, 1);
            out = 'custom_8=' + (payload.custom_8).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
          case codes.CUSTOM_9:
            payload.custom_9 = view.getInt32(0, 1);
            out = 'custom_9=' + (payload.custom_9).toString() + '\n';
            receive.write(out);
            chart.write(out);
            break;
        default:
          break;
        }
      }
      
      var str=receive.getBytes(); //Read the Received string
      
      message_decode(str);
      
      var data = new Uint8Array(str);
      
      // console.log(data);

      //receive.write(str); //Show received string
      //receive.write(str,""Red""); // Shown in red

      return   ;
  }
)() 